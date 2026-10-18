#pragma once
#include "calib/camera_model.hpp"
#include <ceres/rotation.h>

namespace calib {
// Pose blocks: angle-axis (radians), then translation (meters).
// camera maps reference -> camera; target maps target -> reference.
struct RigReprojection {
  std::array<double,3> point;
  std::array<double,2> pixel;
  DistortionModel distortion;
  template<class T> bool operator()(const T* camera,const T* target,const T* p,T* residual) const {
    T source[3]={T(point[0]),T(point[1]),T(point[2])},reference[3],sensor[3];
    ceres::AngleAxisRotatePoint(target,source,reference);
    for(unsigned i=0;i<3;++i) reference[i]+=target[3+i];
    ceres::AngleAxisRotatePoint(camera,reference,sensor);
    for(unsigned i=0;i<3;++i) sensor[i]+=camera[3+i];
    if(sensor[2]<=T(0)) return false;
    const T x=sensor[0]/sensor[2],y=sensor[1]/sensor[2],r2=x*x+y*y;
    T xd,yd;
    if(distortion==DistortionModel::radtan) {
      const T scale=T(1)+p[4]*r2+p[5]*r2*r2;
      xd=x*scale+T(2)*p[6]*x*y+p[7]*(r2+T(2)*x*x);
      yd=y*scale+p[6]*(r2+T(2)*y*y)+T(2)*p[7]*x*y;
    } else {
      T a;
      if(r2<T(1e-8)) a=T(1)+r2*(T(-1.0/3)+r2*(T(1.0/5)-r2/T(7)));
      else { const T r=sqrt(r2); a=atan(r)/r; }
      const T theta2=r2*a*a;
      const T scale=a*(T(1)+theta2*(p[4]+theta2*(p[5]+theta2*(p[6]+theta2*p[7]))));
      xd=x*scale; yd=y*scale;
    }
    residual[0]=p[0]*xd+p[2]-T(pixel[0]);
    residual[1]=p[1]*yd+p[3]-T(pixel[1]);
    return true;
  }
};
} // namespace calib
