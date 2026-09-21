#include <trajectory_generators/braking_rollout.h>
#include <trajectory_generators/path_sweep.h>
#include <cassert>
using namespace trajectory_generators;
int main() {
  // 0.25 m/s, 0.2 m/s^2 braking and 0.7 s response: 0.33125 m to stop.
  double v=.25,d=.7*v;
  for(int i=0;i<30;++i) d+=integrateVelocity(v,0,.1,2,.1);
  assert(std::abs(v)<1e-12 && std::abs(d-.33125)<1e-9);
  // A two-second horizon alone truncates slow braking; a complete tail must not.
  v=.6;d=0;
  for(int i=0;i<20;++i) d+=integrateVelocity(v,0,.1,2,.1);
  assert(v>.19);
  for(int i=0;i<11;++i) d+=integrateVelocity(v,0,.1,2,.1);
  assert(std::abs(v)<1e-12 && std::abs(d-.9)<1e-9);
  v=.2;
  d=integrateVelocity(v,-.1,.1,2,2.0);
  assert(std::abs(v+.1)<1e-12 && std::abs(d-.05)<1e-9);
  assert(chooseMotionOrBrake({{0,0,0,0},{.05,0,0,100}})==1);
  assert(chooseMotionOrBrake({{0,0,0,0},{.05,0,0,-1}})==0);
  assert(chooseMotionOrBrake({{0,0,0,-1},{.05,0,0,-1}})==-1);
  assert(chooseMotionOrBrake({{0,0,0,0},{0,0,.2,10}})==1);
  auto samples=forwardPathSamples({{0,0,-.5},{2,0,-.5}},.2,0,-.08,1.0);
  assert(samples.size()==21);
  assert(std::abs(samples.front()[0]-.2)<1e-9 && std::abs(samples.back()[0]-1.2)<1e-9);
  for(const auto& p:samples) assert(std::abs(p[2]+.08)<1e-9 && std::abs(p[3])<1e-9);
  // Wall at 1.5 m intersects the unchanged body at a future pose, before arrival.
  const std::array<double,3> lo{-.47,-.278,-.428},hi{.47,.278,.232};
  bool hit=false;
  for(const auto& p:samples) hit|=pointInBox({1.5-p[0],0,0-p[2]},lo,hi);
  assert(hit);
  assert(!pointInBox({.2,.30,0},lo,hi)); // No implicit widening.
  assert(!pointInBox({.2,0,.3},lo,hi)); // Antenna is not in the box.
  auto slope=forwardPathSamples({{0,0,-.5},{2,0,-.3}},0,0,0,1);
  assert(std::abs(slope.back()[2]-.1)<1e-9);
  assert(forwardPathSamples({{0,0,0},{0,0,0}},0,0,0,1).empty());
  // Recorded regression: the 10 cm snapped start connector faces -37 deg,
  // but tracking follows the main line at +116 deg. A side wall was falsely
  // hit by the cuboid oriented along that connector.
  std::vector<std::array<double,3>> recorded{
    {1.218,2.766,-.467},{1.246,2.745,-.467},{1.273,2.723,-.467},
    {1.301,2.702,-.467},{1.251,2.803,-.472},{1.2,2.904,-.477},
    {1.149,3.006,-.482},{1.099,3.107,-.487},{1.096,3.305,-.486}};
  auto poses=forwardPathSamples(recorded,1.218,2.766,-.08,1);
  assert(!poses.empty());
  auto hitAt=[&](double x,double y,double z,double yaw,std::array<double,3> obstacle) {
    const double dx=obstacle[0]-x,dy=obstacle[1]-y;
    return pointInBox({dx*std::cos(yaw)+dy*std::sin(yaw),
                       -dx*std::sin(yaw)+dy*std::cos(yaw),obstacle[2]-z},lo,hi);
  };
  assert(hitAt(1.297,2.705,-.08,std::atan2(-.021,.028),{1.799,2.618,-.031}));
  for(const auto& pose:poses)
    assert(!hitAt(pose[0],pose[1],pose[2],pose[3],{1.799,2.618,-.031}));
  // A real obstacle on the forward line must still stop the robot.
  bool real_obstacle=false;
  for(const auto& pose:poses)
    real_obstacle |= hitAt(pose[0],pose[1],pose[2],pose[3],{1.1,3.15,-.03});
  assert(real_obstacle);
}
