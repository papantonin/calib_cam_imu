#include "calib/vi_residual.hpp"
#include <ceres/dynamic_autodiff_cost_function.h>
#include <iostream>
#include <memory>
#include <vector>

namespace {
void near(double actual,double expected,double absolute=2e-5,double relative=2e-5) {
  if(!std::isfinite(actual) || std::abs(actual-expected)>absolute+relative*std::abs(expected))
    throw std::runtime_error("VI residual derivative mismatch: "+std::to_string(actual)+" versus "+std::to_string(expected));
}
template<class Functor> void derivatives(const Functor& functor,std::vector<std::vector<double>> blocks,int rows) {
  ceres::DynamicAutoDiffCostFunction<Functor,4> cost(new Functor(functor));
  std::vector<const double*> pointers;
  std::vector<std::vector<double>> jacobians;
  std::vector<double*> jacobian_pointers;
  for(auto& block:blocks) {
    cost.AddParameterBlock(static_cast<int>(block.size()));
    pointers.push_back(block.data());
    jacobians.emplace_back(static_cast<std::size_t>(rows)*block.size());
  }
  for(auto& jacobian:jacobians) jacobian_pointers.push_back(jacobian.data());
  cost.SetNumResiduals(rows);
  std::vector<double> residual(static_cast<std::size_t>(rows)),plus(residual.size()),minus(residual.size());
  if(!cost.Evaluate(pointers.data(),residual.data(),jacobian_pointers.data())) throw std::runtime_error("Residual evaluation failed");
  constexpr double step=1e-7;
  for(std::size_t block=0;block<blocks.size();++block) for(std::size_t column=0;column<blocks[block].size();++column) {
    const double original=blocks[block][column];
    blocks[block][column]=original+step;
    if(!functor(pointers.data(),plus.data())) throw std::runtime_error("Positive residual perturbation failed");
    blocks[block][column]=original-step;
    if(!functor(pointers.data(),minus.data())) throw std::runtime_error("Negative residual perturbation failed");
    blocks[block][column]=original;
    for(int row=0;row<rows;++row)
      near(jacobians[block][static_cast<std::size_t>(row)*blocks[block].size()+column],
        (plus[static_cast<std::size_t>(row)]-minus[static_cast<std::size_t>(row)])/(2*step));
  }
}
}
int main() {
  try {
    std::vector<std::vector<double>> controls;
    for(unsigned index=0;index<6;++index) {
      const double time=static_cast<double>(index)*.1;
      const std::array<double,3> rotation{.1*std::sin(time*4),.2*std::cos(time*3),.15*std::sin(time*2)};
      std::vector<double> control(7);
      ceres::AngleAxisToQuaternion(rotation.data(),control.data());
      control[4]=.1*std::sin(time*3);control[5]=.2*std::cos(time*4);control[6]=-1+.1*std::sin(time*2);
      controls.push_back(control);
    }
    for(auto distortion:{calib::DistortionModel::radtan,calib::DistortionModel::equidistant}) {
      const calib::vi_detail::VisualResidual visual{{.1,.2,0},{650,500},{500,505,640,480,-.03,.005,.001,-.001},distortion,.37,.1,.8};
      derivatives(visual,{controls.begin(),controls.begin()+4},2);
    }
    std::vector<std::vector<double>> blocks{{.93,.12,-.19,.24,.03,-.04,.06},{.01,-.02,.03,.1,-.1,.05},{0,0,-9.80665},{.01}};
    blocks.insert(blocks.end(),controls.begin(),controls.end());
    for(double time:{.047,.1099,.1101,.173}) {
      calib::vi_detail::ImuResidual imu{{.1,.2,-.3,.2,-.4,9.7},time,.1,.02,.2,0,3};
      derivatives(imu,blocks,6);
    }
    auto variable_blocks=blocks;
    variable_blocks.insert(variable_blocks.begin()+4,{.012,-.018,.027,.11,-.08,.03});
    for(double time:{.047,.1099,.1101,.173}) {
      calib::vi_detail::ImuResidual imu{{.1,.2,-.3,.2,-.4,9.7},time,.1,.02,.2,0,3,.37,true};
      derivatives(imu,variable_blocks,6);
    }
    calib::vi_detail::BiasRandomWalkResidual random_walk{2e-4,3e-3};
    derivatives(random_walk,{{.01,-.02,.03,.1,-.1,.05},{.012,-.018,.027,.11,-.08,.03}},6);
    std::vector<const double*> pointers;
    for(const auto& block:blocks) pointers.push_back(block.data());
    calib::vi_detail::ImuResidual left{{},.11-1e-10,.1,1,1,0,3},right=left;
    right.time=.11+1e-10;
    std::array<double,6> first{},second{};
    left(pointers.data(),first.data());right(pointers.data(),second.data());
    for(unsigned index=0;index<6;++index) near(first[index],second[index],1e-7,1e-7);
    for(double time:{.009999,.310001}) {
      auto outside=left;
      outside.time=time;
      if(outside(pointers.data(),first.data())) throw std::runtime_error("IMU residual extrapolates outside declared spline support");
    }
    std::cout<<"VI visual/inertial and bias random-walk Jacobians, lever arm, variable biases, gravity, time shift, segment crossing and support bounds passed\n";
    return 0;
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
