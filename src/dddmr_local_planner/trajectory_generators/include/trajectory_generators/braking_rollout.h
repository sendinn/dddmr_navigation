#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <array>
namespace trajectory_generators {
// Exact 1D integration: finite deceleration, including reversal through zero.
inline double integrateVelocity(double& velocity, double target, double acceleration,
                                double deceleration_ratio, double dt) {
  double remaining=dt, displacement=0;
  for(int phase=0;phase<2 && remaining>0;++phase) {
    const double phase_target=velocity*target<0 ? 0 : target;
    const double rate=acceleration*(std::abs(phase_target)<std::abs(velocity)?deceleration_ratio:1);
    const double delta=phase_target-velocity;
    const double duration=std::min(remaining,std::abs(delta)/rate);
    const double next=velocity+std::copysign(rate*duration,delta);
    displacement+=.5*(velocity+next)*duration;
    velocity=duration>=std::abs(delta)/rate?phase_target:next;
    remaining-=duration;
    if(velocity==target) break;
  }
  return displacement+velocity*remaining;
}
inline bool zeroCommand(double x,double y,double yaw) {
  return std::abs(x)<1e-6 && std::abs(y)<1e-6 && std::abs(yaw)<1e-6;
}
// Never choose stationary fallback over a safe moving command solely on cost.
inline int chooseMotionOrBrake(const std::vector<std::array<double,4>>& candidates) {
  int moving=-1, brake=-1;
  for(size_t i=0;i<candidates.size();++i) {
    const auto& c=candidates[i];
    if(!std::isfinite(c[3]) || c[3]<0) continue;
    int& best=zeroCommand(c[0],c[1],c[2])?brake:moving;
    if(best<0 || c[3]<candidates[best][3]) best=static_cast<int>(i);
  }
  return moving>=0?moving:brake;
}
}
