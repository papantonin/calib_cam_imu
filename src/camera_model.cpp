#include "calib/camera_model.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace calib {
namespace {
template<class Range> bool finite(const Range& values) {
  return std::ranges::all_of(values,[](double value){return std::isfinite(value);});
}
}
CameraModel::CameraModel(DistortionModel distortion,Parameters parameters)
    : distortion_(distortion),parameters_(parameters) {
  if(distortion!=DistortionModel::radtan && distortion!=DistortionModel::equidistant)
    throw std::invalid_argument("Unknown distortion model");
  if(!finite(parameters) || parameters[0]<=0 || parameters[1]<=0)
    throw std::invalid_argument("Camera parameters must be finite with positive focal lengths");
}

std::expected<Projection,ProjectionError>
CameraModel::project(const std::array<double,3>& point) const noexcept {
  if(!finite(point)) return std::unexpected(ProjectionError::nonfinite_point);
  if(point[2]<=0) return std::unexpected(ProjectionError::nonpositive_depth);
  const double x=point[0]/point[2], y=point[1]/point[2];
  const double r2=x*x+y*y;
  if(!std::isfinite(r2)) return std::unexpected(ProjectionError::numerical_overflow);
  const auto& p=parameters_;
  double xd{},yd{},j00{},j01{},j11{};
  Projection result{};
  auto& jp=result.parameter_jacobian;
  if(distortion_==DistortionModel::radtan) {
    const double r4=r2*r2, xy=x*y;
    const double radial=1+p[4]*r2+p[5]*r4;
    const double gradient=2*(p[4]+2*p[5]*r2);
    xd=x*radial+2*p[6]*xy+p[7]*(r2+2*x*x);
    yd=y*radial+p[6]*(r2+2*y*y)+2*p[7]*xy;
    j00=radial+x*x*gradient+2*p[6]*y+6*p[7]*x;
    j01=xy*gradient+2*p[6]*x+2*p[7]*y;
    j11=radial+y*y*gradient+6*p[6]*y+2*p[7]*x;
    jp[0]={xd,0,1,0,p[0]*x*r2,p[0]*x*r4,p[0]*2*xy,p[0]*(r2+2*x*x)};
    jp[1]={0,yd,0,1,p[1]*y*r2,p[1]*y*r4,p[1]*(r2+2*y*y),p[1]*2*xy};
  } else {
    const double r=std::sqrt(r2), theta=std::atan(r), t2=theta*theta;
    const double polynomial=1+t2*(p[4]+t2*(p[5]+t2*(p[6]+t2*p[7])));
    const double derivative=p[4]+t2*(2*p[5]+t2*(3*p[6]+t2*4*p[7]));
    // a=atan(r)/r; q=2*d(a)/d(r²). Analytic limits avoid cancellation
    // around the optical axis. Remainders here are O(r^8), below double epsilon.
    const double a=r2<1e-8 ? 1+r2*(-1.0/3+r2*(1.0/5-r2/7)) : theta/r;
    const double q=r2<1e-8 ? -2.0/3+r2*(4.0/5+r2*(-6.0/7+r2*8.0/9))
                                : (1/(1+r2)-a)/r2;
    const double scale=a*polynomial;
    const double gradient=q*polynomial+2*a*a*derivative/(1+r2);
    xd=x*scale; yd=y*scale;
    j00=scale+x*x*gradient; j01=x*y*gradient; j11=scale+y*y*gradient;
    jp[0]={xd,0,1,0,0,0,0,0}; jp[1]={0,yd,0,1,0,0,0,0};
    double power=t2;
    for(unsigned i=4;i<8;++i) {
      jp[0][i]=p[0]*x*a*power; jp[1][i]=p[1]*y*a*power;
      power*=t2;
    }
  }
  result.pixel={p[0]*xd+p[2],p[1]*yd+p[3]};
  auto& j=result.point_jacobian;
  j[0][0]=p[0]*j00/point[2]; j[0][1]=p[0]*j01/point[2];
  j[1][0]=p[1]*j01/point[2]; j[1][1]=p[1]*j11/point[2];
  for(unsigned row=0;row<2;++row) j[row][2]=-j[row][0]*x-j[row][1]*y;
  if(!finite(result.pixel) || !finite(j[0]) || !finite(j[1]) || !finite(jp[0]) || !finite(jp[1]))
    return std::unexpected(ProjectionError::numerical_overflow);
  return result;
}
} // namespace calib
