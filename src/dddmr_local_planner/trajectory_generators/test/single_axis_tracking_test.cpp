#include <trajectory_generators/single_axis_tracking.h>
#include <cassert>
#include <dddmr_sys_core/motion_timestamp.h>
#include <trajectory_generators/rotation_speed.h>
#include <string>
using namespace trajectory_generators;
int main() {
  const double pi = std::acos(-1.0);
  assert(rotationSpeed(pi/2, .12, .6) == .6);
  assert(std::abs(rotationSpeed(pi/6, .12, .6) - pi/6) < 1e-9);
  assert(std::abs(rotationSpeed(-pi/9, .12, .6) - pi/9) < 1e-9);
  assert(rotationSpeed(.01, .12, .6) == .12);
  assert(rotationSpeed(NAN, .12, .6) == 0);
  double previous = .6;
  for (double angle = 1.5; angle >= 0; angle -= .01) {
    const double speed = rotationSpeed(angle, .12, .6);
    assert(speed <= previous && speed >= .12 && speed <= .6);
    previous = speed;
  }
  auto e = trackingErrors({{0,0},{2,0}}, 0.5, 0.3, 0);
  assert(e.valid && std::abs(e.lateral+0.3)<1e-9 && e.forward>0);
  auto rotated = trackingErrors({{0,0},{0,2}}, 0.3, 0.5, pi/2);
  assert(rotated.valid && std::abs(rotated.lateral-0.3)<1e-9);
  assert(!trackingErrors({{0,0},{0,0}},0,0,0).valid);
  auto wrap = trackingErrors({{0,0},{-2,0}},0,0,-pi+0.01);
  assert(std::abs(wrap.heading+0.01)<1e-9);
  auto corner = trackingErrors({{0,0},{1,0},{1,1}},1,0,0);
  assert(std::abs(corner.heading-pi/2)<1e-9);

  // Reproduces a backward start connector before a forward corridor.
  auto connector = trackingErrors({{0,0},{-.04,0},{-.08,0},{-.12,0},
                                   {-.12,.2},{-.12,1.0}},0,0,0);
  assert(connector.valid && std::abs(connector.heading-pi/2)<1e-9);
  assert(std::abs(connector.lateral-.12)<1e-9); // Return to main line, not connector.
  auto dense = trackingErrors({{0,0},{0,0},{-.12,0},{-.12,.1},{-.12,.2},
                               {-.12,.3},{-.12,.4},{-.12,1}},0,0,0);
  assert(dense.valid && std::abs(dense.heading-connector.heading)<1e-9);
  auto end = trackingErrors({{0,0},{0,1}},0,1,pi/2);
  assert(end.valid && std::abs(end.heading)<1e-9);
  assert(!trackingErrors({{0,0},{1,0}},0,0,0,0).valid);
  assert(!trackingErrors({{0,0},{NAN,1}},0,0,0).valid);

  auto targets = singleAxisTargets(-.2,.2,.05,5,1,.2);
  assert(targets.size()==5 && targets.front()==.05 && targets.back()==.2);
  auto reverse = singleAxisTargets(-.2,.2,.05,5,-1,.2);
  assert(reverse.size()==5 && reverse.front()==-.05 && reverse.back()==-.2);
  assert(singleAxisTargets(-.2,.2,.05,5,1,.04).empty()); // Speed zone below floor: stop.
  assert(singleAxisTargets(0,.2,.05,5,-1,.2).empty()); // Reverse disabled.
  auto turn = singleAxisTargets(-.6,.6,.1,10,1,.26);
  assert(turn.front()==.1 && std::abs(turn.back()-.26)<1e-9);
  assert(singleAxisTargets(-.2,.2,NAN,5,1,.2).empty());

  SingleAxisTracking timed;
  TrackingErrors te{true,.8,0,1};
  assert(timed.choose(te,{0,0,0},1,true,.52,.26,.3,.15,true)==-1);
  assert(timed.choose(te,{0,0,0},2,true,.52,.26,.3,.15,true)==-1);
  assert(timed.choose(te,{0,0,0},3,true,.52,.26,.3,.15,true)==2);
  assert(timed.choose(te,{.06,0,.6},4,true,.52,.26,.3,.15,true)==2);
  te.heading=-.8;
  assert(timed.choose(te,{.06,0,.6},5,true,.52,.26,.3,.15,true)==-1);

  SingleAxisTracking policy;
  int64_t stamp=0;
  auto choose = [&](TrackingErrors err, std::array<double,3> v, bool fresh=true) {
    return policy.choose(err,v,++stamp,fresh,.174533,.087267,.10,.05);
  };
  e.heading=0.5;
  assert(choose(e,{0,0,0})==-1);
  // Repeated odometry cannot satisfy the three-sample stopped condition.
  assert(policy.choose(e,{0,0,0},stamp,true,.174533,.087267,.10,.05)==-1);
  assert(choose(e,{0,0,0})==-1);
  assert(choose(e,{0,0,0})==2 && policy.sign()==1);
  e.heading=.12;
  assert(choose(e,{0,0,.08})==2); // Hysteresis keeps turning until 5 degrees.
  e.heading=.02;
  assert(choose(e,{0,0,.08})==-1); // Brake rotation before strafe.
  assert(choose(e,{0,0,0})==-1);
  assert(choose(e,{0,0,0})==-1);
  assert(choose(e,{0,0,0})==1 && policy.sign()==-1);
  e.lateral=-.07;
  assert(choose(e,{0,-.08,0})==1); // Keep strafing to 5 cm.
  e.lateral=-.04;
  assert(choose(e,{0,-.08,0})==-1);
  assert(choose(e,{0,0,0})==-1);
  assert(choose(e,{0,0,0})==-1);
  assert(choose(e,{0,0,0})==0);
  assert(choose(e,{.08,0,0})==0);
  e.heading=.3;
  assert(choose(e,{.08,0,0})==-1); // New corner preempts X, but brakes first.
  assert(choose(e,{0,0,0},false)==-1); // Stale odom resets stage.
  assert(std::string(policy.stopReason()).find("TF") != std::string::npos);
  assert(choose(e,{0,0,0})==-1);
  assert(choose(e,{0,0,0})==-1);
  assert(choose(e,{0,0,0})==2);
  e.heading=-.3;
  assert(choose(e,{0,0,.1})==-1); // Direction reversal also waits for stop.
  assert(std::string(policy.stopReason()).find("切换轴或方向") != std::string::npos);
  assert(choose(e,{0,0,0})==-1);
  assert(choose(e,{0,0,0})==-1);
  assert(choose(e,{0,0,0})==2 && policy.sign()==-1);
  // Replay the small future ages observed during straight-line motion.
  using dddmr_sys_core::motionTimestampFresh;
  assert(motionTimestampFresh(1, -.020));
  assert(!motionTimestampFresh(1, -.020001));
  assert(motionTimestampFresh(1, .499));
  assert(!motionTimestampFresh(1, .5));
  assert(!motionTimestampFresh(0, 0));
  assert(!motionTimestampFresh(-1, 0));
  assert(!motionTimestampFresh(1, NAN));
  assert(!motionTimestampFresh(1, INFINITY));
  SingleAxisTracking jitter;
  TrackingErrors straight{true, 0, 0, 1};
  int64_t tick = 100;
  auto sample = [&](double age, double vx=0) {
    ++tick;
    return jitter.choose(straight, {vx,0,0}, tick,
      motionTimestampFresh(tick, age), .52,.17,.3,.15);
  };
  assert(sample(0)==-1);
  assert(sample(-.013)==-1);
  assert(sample(-.005)==0);
  for (double age : {-.013, .01, -.005, -.003, -.012, -.001})
    assert(sample(age, .255)==0);
  assert(sample(-.021, .255)==-1);
  assert(sample(0, .255)==-1);
  assert(sample(0)==-1);
  assert(sample(-.013)==-1);
  assert(sample(0)==0);
  assert(sample(.5, .255)==-1);
  assert(sample(0)==-1);
  assert(sample(0)==-1);
  tick -= 10;
  assert(sample(0)==-1);
  assert(std::string(jitter.stopReason()).find("倒退") != std::string::npos);
  assert(sample(0)==-1);
  assert(jitter.choose(straight,{0,0,0},tick,true,.52,.17,.3,.15)==-1);
  assert(sample(0)==-1);
  assert(sample(0)==0);
  tick -= 10;
  assert(sample(0,.255)==-1);
  // Equal enter/exit thresholds: >10 degrees turns; <=10 degrees brakes
  // the turn and confirms stopped samples before resuming X.
  SingleAxisTracking ten_degree;
  const double ten = pi/18;
  TrackingErrors heading_boundary{true,ten+.001,0,1};
  for (int i=1;i<=3;++i)
    assert(ten_degree.choose(heading_boundary,{0,0,0},i,true,ten,ten,.3,.15)==(i<3?-1:2));
  heading_boundary.heading=ten;
  assert(ten_degree.choose(heading_boundary,{0,0,.12},4,true,ten,ten,.3,.15)==-1);
  for (int i=5;i<=7;++i)
    assert(ten_degree.choose(heading_boundary,{0,0,0},i,true,ten,ten,.3,.15)==(i<7?-1:0));
  heading_boundary.heading=-ten-.001;
  assert(ten_degree.choose(heading_boundary,{.1,0,0},8,true,ten,ten,.3,.15)==-1);
}
