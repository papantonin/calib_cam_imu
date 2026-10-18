#include "calib/core.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>

namespace {
void require(bool condition,const char* message) {
  if(!condition) throw std::runtime_error(message);
}
void near(double actual,double expected,double absolute=1e-10,double relative=1e-11) {
  if(!std::isfinite(actual) || std::abs(actual-expected)>absolute+relative*std::abs(expected))
    throw std::runtime_error("Mismatch: "+std::to_string(actual)+" != "+std::to_string(expected));
}
template<class F> void fails(F operation) {
  bool failed=false;
  try { operation(); } catch(const std::exception&) { failed=true; }
  require(failed,"invalid camera accepted");
}
void derivatives(const calib::CameraModel& model,const std::array<double,3>& point) {
  const auto value=model.project(point);
  require(value.has_value(),"valid point rejected");
  for(unsigned i=0;i<3;++i) {
    // Relative to depth to avoid crossing the camera plane.
    const double step=1e-5*point[2];
    auto plus=point,minus=point;
    plus[i]+=step; minus[i]-=step;
    const auto a=model.project(plus),b=model.project(minus);
    require(a.has_value() && b.has_value(),"invalid perturbed point");
    for(unsigned row=0;row<2;++row)
      near(value->point_jacobian[row][i],(a->pixel[row]-b->pixel[row])/(2*step),2e-6,2e-7);
  }
  for(unsigned i=0;i<8;++i) {
    auto plus=model.parameters(),minus=plus;
    const double step=1e-5*std::max(1.,std::abs(plus[i]));
    plus[i]+=step; minus[i]-=step;
    const auto a=calib::CameraModel(model.distortion(),plus).project(point);
    const auto b=calib::CameraModel(model.distortion(),minus).project(point);
    require(a.has_value() && b.has_value(),"invalid perturbed parameters");
    for(unsigned row=0;row<2;++row)
      near(value->parameter_jacobian[row][i],(a->pixel[row]-b->pixel[row])/(2*step),2e-6,2e-7);
  }
}
}
int main() {
  using namespace calib;
  try {
    const CameraModel::Parameters ideal{400,420,320,240,0,0,0,0};
    const CameraModel pinhole(DistortionModel::radtan,ideal);
    auto result=pinhole.project({1,-.5,2}).value();
    near(result.pixel[0],520); near(result.pixel[1],135);
    near(result.point_jacobian[0][0],200); near(result.point_jacobian[0][2],-100);
    near(result.point_jacobian[1][1],210); near(result.point_jacobian[1][2],52.5);
    const CameraModel radial(DistortionModel::radtan,{400,420,320,240,.1,.02,.003,-.004});
    result=radial.project({1,-.5,2}).value();
    near(result.pixel[0],525.040625); near(result.pixel[1],132.484921875);
    const CameraModel fisheye(DistortionModel::equidistant,ideal);
    result=fisheye.project({1,0,1}).value();
    near(result.pixel[0],320+400*std::numbers::pi/4);
    near(result.point_jacobian[0][0],200); near(result.point_jacobian[0][2],-200);
    require(result.pixel[0]!=pinhole.project({1,0,1})->pixel[0],"zero-coefficient fisheye became pinhole");

    std::mt19937_64 rng(20261017);
    std::uniform_real_distribution<double> unit(-1,1);
    unsigned checked=0;
    for(const auto kind:{DistortionModel::radtan,DistortionModel::equidistant}) {
      const CameraModel model(kind,{410,397,253,249,.01,-.002,.0003,-.00002});
      const auto axis=model.project({0,0,2}).value();
      near(axis.pixel[0],253); near(axis.pixel[1],249);
      near(axis.point_jacobian[0][0],205); near(axis.point_jacobian[1][1],198.5);
      for(unsigned row=0;row<2;++row)
        for(unsigned i=4;i<8;++i) near(axis.parameter_jacobian[row][i],0);
      // Include both sides of the near-axis series boundary, every quadrant,
      // and exactly the axis; these are also differentiated, not just projected.
      for(double radius:{0.,1e-12,1e-8,1e-4*(1-1e-8),1e-4*(1+1e-8),.2,1.,5.}) {
        derivatives(model,{radius,0,1}); derivatives(model,{-radius,radius,1});
        checked+=2;
      }
      for(unsigned n=0;n<300;++n) {
        const double z=std::exp(2*unit(rng));
        const std::array<double,3> point{3*z*unit(rng),3*z*unit(rng),z};
        derivatives(model,point); ++checked;
        const auto original=model.project(point).value();
        auto scaled=point;
        for(auto& coordinate:scaled) coordinate*=7;
        const auto other=model.project(scaled).value();
        for(unsigned row=0;row<2;++row) {
          near(other.pixel[row],original.pixel[row]);
          for(unsigned i=0;i<3;++i) near(7*other.point_jacobian[row][i],original.point_jacobian[row][i]);
        }
      }
      // Outside the sensor is still a valid projection; clipping belongs to the caller.
      require(model.project({1,1,.1}).has_value(),"outside-image point clipped");
      require(model.project({1,1,0}).error()==ProjectionError::nonpositive_depth,"zero depth");
      require(model.project({0,0,-1}).error()==ProjectionError::nonpositive_depth,"behind camera");
      const double nan=std::numeric_limits<double>::quiet_NaN();
      const double inf=std::numeric_limits<double>::infinity();
      for(unsigned i=0;i<3;++i) for(const double bad:{nan,inf}) {
        std::array<double,3> point{0,0,1}; point[i]=bad;
        require(model.project(point).error()==ProjectionError::nonfinite_point,"nonfinite point");
      }
      require(model.project({1e308,0,1e-308}).error()==ProjectionError::numerical_overflow,"overflow accepted");
      for(unsigned i=0;i<8;++i) {
        auto p=ideal; p[i]=nan; fails([&]{CameraModel invalid(kind,p);});
      }
      for(unsigned i=0;i<2;++i) for(double bad:{0.,-1.}) {
        auto p=ideal; p[i]=bad; fails([&]{CameraModel invalid(kind,p);});
      }
      CameraCalibration camera{"cam0","pinhole",kind==DistortionModel::radtan?"radtan":"equidistant",
        {410,397,253,249},{.01,-.002,.0003,-.00002},512,512,1e-5,true};
      require(make_camera_model(camera).parameters()==model.parameters(),"camchain parameter order");
      require(make_camera_model(camera).distortion()==kind,"camchain model mapping");
      camera.intrinsics.pop_back(); fails([&]{(void)make_camera_model(camera);});
    }
    fails([&]{CameraModel invalid(static_cast<DistortionModel>(99),ideal);});
    const double angle=89.*std::numbers::pi/180;
    const auto horizon=fisheye.project({std::sin(angle),0,std::cos(angle)}).value();
    near(horizon.pixel[0],320+400*angle);
    near(horizon.point_jacobian[0][0],400*std::cos(angle),1e-8);
    near(horizon.point_jacobian[0][2],-400*std::sin(angle),1e-8);
    std::cout<<checked<<" cases: all 11 derivative columns, model limits and camchain adapter passed\n";
    return 0;
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
