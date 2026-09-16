#include <cassert>
#include <cmath>
#include <limits>
#include <mcl_3dl/odom_history.h>

using namespace mcl_3dl;
State6DOF state(double t, double x, double y, double angle)
{
  Quat q; q.setRPY(Vec3(0, 0, angle));
  return State6DOF(Vec3(x,y,0), q, t);
}
int main()
{
  OdomHistory h; State6DOF out;
  assert(!h.sample(1, out));
  assert(h.push(state(1,0,0,3.1)));
  assert(h.push(state(1.2,0.2,0.4,-3.1)));
  assert(!h.push(state(1.1,99,99,0)));
  assert(!h.push(state(1.2,99,99,0)));
  assert(!h.sample(0.9,out) && !h.sample(1.3,out));
  assert(!h.sample(std::numeric_limits<double>::quiet_NaN(),out));
  assert(h.sample(1.1,out));
  assert(std::abs(out.pos_.x_-0.1)<1e-6 && std::abs(out.pos_.y_-0.2)<1e-6);
  assert(std::abs(std::abs(out.rot_.getRPY().z_)-M_PI)<1e-5);
  assert(h.sample(1.2,out) && std::abs(out.pos_.x_-0.2)<1e-6);
  assert(h.push(state(2,1,0,0)));
  assert(!h.sample(1.6,out)); // Missing stream: no invented interpolation.
  for (int i=1;i<=50;++i) assert(h.push(state(2+i*0.1,i,0,0)));
  assert(!h.sample(1.2,out)); // History is bounded.

  // Delayed scan on a translating/rotating robot: map->odom must use
  // the scan's odometry, not the latest received pose.
  OdomHistory moving;
  for (int i=0;i<=10;++i) moving.push(state(10+i*0.1,i*0.01,0,i*0.02));
  State6DOF scan, latest;
  assert(moving.sample(10.8,scan) && moving.sample(11,latest));
  const Vec3 map_offset(2,3,0);
  const Vec3 localized_scan = scan.pos_ + map_offset;
  const auto correct_offset = localized_scan - scan.pos_;
  assert((correct_offset-map_offset).norm()<1e-6);
  assert((localized_scan-latest.pos_-map_offset).norm()>0.019);
  assert(std::abs(scan.rot_.getRPY().z_-0.16)<1e-6);
}
