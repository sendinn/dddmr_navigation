#pragma once
#include <trajectory_generators/single_axis_tracking.h>
#include <array>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
namespace trajectory_generators {
// Pose samples on the ordered forward path. Preserve base-to-ground height;
// a ground path's Z must not be used directly as the cuboid's base_link Z.
inline std::vector<std::array<double,4>> forwardPathSamples(
    const std::vector<std::array<double,3>>& path, double x,double y,double z,
    double lookahead,double step=.05,double heading_lookahead=.5) {
  std::vector<std::array<double,4>> out;
  if(path.size()<2 || !(lookahead>0) || !(step>0) || !std::isfinite(lookahead) ||
     !std::isfinite(step) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return out;
  std::vector<double> arc(path.size(),0);
  double best=std::numeric_limits<double>::infinity(),nearest=0,ground=0;
  for(size_t i=0;i<path.size();++i) {
    for(double v:path[i]) if(!std::isfinite(v)) return {};
    if(!i) continue;
    const auto &a=path[i-1], &b=path[i];
    const double dx=b[0]-a[0],dy=b[1]-a[1],len=std::hypot(dx,dy);
    arc[i]=arc[i-1]+len;
    if(len<1e-8) continue;
    const double t=std::clamp(((x-a[0])*dx+(y-a[1])*dy)/(len*len),0.,1.);
    const double d=std::hypot(a[0]+t*dx-x,a[1]+t*dy-y);
    if(d<best) {best=d;nearest=arc[i-1]+t*len;ground=a[2]+t*(b[2]-a[2]);}
  }
  if(!std::isfinite(best)) return out;
  std::vector<std::array<double,2>> heading_path;
  for (const auto& p:path) heading_path.push_back({p[0],p[1]});
  const double end=std::min(arc.back(),nearest+lookahead);
  const int count=std::max(1,static_cast<int>(std::ceil((end-nearest)/step)));
  for(int k=0;k<=count;++k) {
    const double distance=nearest+(end-nearest)*k/count;
    for(size_t i=1;i<path.size();++i) {
      const double len=arc[i]-arc[i-1];
      if(len<1e-8 || arc[i]+1e-9<distance) continue;
      const auto &a=path[i-1],&b=path[i];
      const double t=std::clamp((distance-arc[i-1])/len,0.,1.);
      const double sx=a[0]+t*(b[0]-a[0]), sy=a[1]+t*(b[1]-a[1]);
      // Match the controller's forward reference, not a short snapped connector
      // tangent that the robot will never align with during path following.
      const auto reference=trackingErrors(heading_path,sx,sy,0,heading_lookahead);
      if (!reference.valid) return {};
      out.push_back({sx,sy,z+a[2]+t*(b[2]-a[2])-ground,reference.heading});
      break;
    }
  }
  return out;
}
inline bool pointInBox(const std::array<double,3>& p,const std::array<double,3>& lo,
                       const std::array<double,3>& hi) {
  for(int i=0;i<3;++i) if(!std::isfinite(p[i]) || p[i]<lo[i] || p[i]>hi[i]) return false;
  return true;
}
}
