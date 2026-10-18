#include "calib/rig_residual.hpp"
#include <ceres/autodiff_cost_function.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

void near(double a,double b,double absolute=2e-5,double relative=2e-6) {
  if(!std::isfinite(a) || !std::isfinite(b) || std::abs(a-b)>absolute+relative*std::abs(b))
    throw std::runtime_error("Rig derivative mismatch");
}
int main() {
  try {
    std::mt19937_64 random(83); std::uniform_real_distribution<double> unit(-1.,1.);
    for(const auto kind:{calib::DistortionModel::radtan,calib::DistortionModel::equidistant}) {
      for(unsigned n=0;n<70;++n) {
        std::array<double,6> camera{.1*unit(random),.1*unit(random),.1*unit(random),-.12,.003,.002};
        std::array<double,6> target{.4*unit(random),.4*unit(random),.4*unit(random),-.1,.05,.8};
        std::array<double,8> p{400,410,320,240,.02,-.002,.0001,-.00001};
        if(n==0) { camera={}; target={0,0,0,0,0,1}; }
        if(n==1) target[0]=3.141592653589793;
        const std::array<double,3> point=n==0?std::array<double,3>{0,0,0}:std::array<double,3>{.1*unit(random),.1*unit(random),0};
        ceres::AutoDiffCostFunction<calib::RigReprojection,2,6,6,8> cost(new calib::RigReprojection{point,{10,20},kind});
        std::array<double,2> residual; std::array<double,12> j0,j1; std::array<double,16> j2;
        const double* parameters[]{camera.data(),target.data(),p.data()}; double* jacobians[]{j0.data(),j1.data(),j2.data()};
        if(!cost.Evaluate(parameters,residual.data(),jacobians)) throw std::runtime_error("Valid rig residual refused");
        if(n==0) {
          const auto projected=calib::CameraModel(kind,p).project({0,0,1}).value();
          near(residual[0],projected.pixel[0]-10,1e-12,0); near(residual[1],projected.pixel[1]-20,1e-12,0);
        }
        double* mutable_parameters[]{camera.data(),target.data(),p.data()};
        const unsigned sizes[]{6,6,8};
        for(unsigned block=0;block<3;++block) for(unsigned column=0;column<sizes[block];++column) {
          auto& value=mutable_parameters[block][column]; const double saved=value;
          const double step=1e-6*std::max(1.,std::abs(value));
          std::array<double,2> plus,minus;
          value=saved+step; if(!cost.Evaluate(parameters,plus.data(),nullptr)) throw std::runtime_error("Invalid FD plus");
          value=saved-step; if(!cost.Evaluate(parameters,minus.data(),nullptr)) throw std::runtime_error("Invalid FD minus");
          value=saved;
          for(unsigned row=0;row<2;++row) near(jacobians[block][row*sizes[block]+column],(plus[row]-minus[row])/(2*step));
        }
      }
      const calib::RigReprojection residual{{0,0,0},{0,0},kind};
      const std::array<double,6> camera{},behind{0,0,0,0,0,-1};
      const std::array<double,8> p{400,410,320,240,0,0,0,0}; double result[2];
      if(residual(camera.data(),behind.data(),p.data(),result)) throw std::runtime_error("Behind-camera point accepted");
    }
    std::cout<<"140 cases, 20 Jacobian columns, optical axis and pi rotation passed\n"; return 0;
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
