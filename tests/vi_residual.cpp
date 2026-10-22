#include "calib/vi_spline.hpp"
#include <algorithm>
#include <iostream>
#include <random>
#include <string>

namespace {
using Knot=std::array<double,7>;
using Controls=std::array<Knot,5>;
using State=calib::vi_detail::SplineState<double>;

void near(double actual,double expected,double absolute=2e-6,double relative=2e-6) {
  if(!std::isfinite(actual) || !std::isfinite(expected) || std::abs(actual-expected)>absolute+relative*std::abs(expected))
    throw std::runtime_error("VI spline mismatch: "+std::to_string(actual)+" versus "+std::to_string(expected));
}

State sample(const Controls& controls,double phase,double interval,unsigned start=0) {
  const double* knots[]{controls[start].data(),controls[start+1].data(),controls[start+2].data(),controls[start+3].data()};
  return calib::vi_detail::evaluate(knots,phase,interval);
}

std::array<double,16> flatten(const State& state) {
  std::array<double,16> values{};
  std::copy(state.rotation.begin(),state.rotation.end(),values.begin());
  std::copy(state.position.begin(),state.position.end(),values.begin()+4);
  std::copy(state.acceleration.begin(),state.acceleration.end(),values.begin()+7);
  std::copy(state.omega.begin(),state.omega.end(),values.begin()+10);
  std::copy(state.alpha.begin(),state.alpha.end(),values.begin()+13);
  return values;
}

void derivative_checks(Controls controls,double phase,double interval) {
  const double step=1e-4*interval;
  const auto state=sample(controls,phase,interval);
  const auto plus=sample(controls,phase+step/interval,interval);
  const auto minus=sample(controls,phase-step/interval,interval);
  std::array<double,4> rotation_derivative,body_derivative;
  for(unsigned component=0;component<4;++component)
    rotation_derivative[component]=(plus.rotation[component]-minus.rotation[component])/(2*step);
  const auto inverse=calib::vi_detail::conjugate(state.rotation);
  ceres::QuaternionProduct(inverse.data(),rotation_derivative.data(),body_derivative.data());
  for(unsigned axis=0;axis<3;++axis) {
    near(state.omega[axis],2*body_derivative[axis+1],2e-6,2e-6);
    near(state.alpha[axis],(plus.omega[axis]-minus.omega[axis])/(2*step),2e-5,2e-6);
    near(state.acceleration[axis],(plus.position[axis]-2*state.position[axis]+minus.position[axis])/(step*step),2e-4,2e-6);
  }
  using Jet=ceres::Jet<double,29>;
  std::array<std::array<Jet,7>,4> differentiated;
  const Jet* pointers[4];
  for(unsigned control=0;control<4;++control) {
    pointers[control]=differentiated[control].data();
    for(unsigned component=0;component<7;++component)
      differentiated[control][component]=Jet(controls[control][component],static_cast<int>(control*7+component));
  }
  const auto jets=calib::vi_detail::evaluate(pointers,Jet(phase,28),interval);
  std::array<Jet,16> outputs;
  std::copy(jets.rotation.begin(),jets.rotation.end(),outputs.begin());
  std::copy(jets.position.begin(),jets.position.end(),outputs.begin()+4);
  std::copy(jets.acceleration.begin(),jets.acceleration.end(),outputs.begin()+7);
  std::copy(jets.omega.begin(),jets.omega.end(),outputs.begin()+10);
  std::copy(jets.alpha.begin(),jets.alpha.end(),outputs.begin()+13);
  for(unsigned parameter=0;parameter<29;++parameter) {
    auto upper=controls,lower=controls;
    double upper_phase=phase,lower_phase=phase;
    const double difference_step=1e-6;
    if(parameter<28) {
      upper[parameter/7][parameter%7]+=difference_step;
      lower[parameter/7][parameter%7]-=difference_step;
    } else {
      upper_phase+=difference_step;
      lower_phase-=difference_step;
    }
    const auto upper_values=flatten(sample(upper,upper_phase,interval));
    const auto lower_values=flatten(sample(lower,lower_phase,interval));
    for(unsigned output=0;output<outputs.size();++output)
      near(outputs[output].v[static_cast<int>(parameter)],(upper_values[output]-lower_values[output])/(2*difference_step),2e-5,3e-6);
  }
}

void analytic_checks() {
  Controls controls{};
  const double interval=.2;
  for(unsigned control=0;control<controls.size();++control) {
    const double time=static_cast<double>(control)*interval;
    const std::array<double,3> angle{0,0,.7*time};
    ceres::AngleAxisToQuaternion(angle.data(),controls[control].data());
    controls[control][4]=2*time+.3;
    controls[control][5]=time*time;
    controls[control][6]=time*time*time;
  }
  for(const double phase:{0.,.2,.7,1.}) {
    const auto state=sample(controls,phase,interval);
    const double time=(1+phase)*interval;
    near(state.position[0],2*time+.3,1e-12,0);
    near(state.position[1],time*time+interval*interval/3,1e-12,0);
    near(state.position[2],time*time*time+time*interval*interval,1e-12,0);
    near(state.acceleration[0],0,1e-12,0);
    near(state.acceleration[1],2,1e-12,0);
    near(state.acceleration[2],6*time,1e-12,0);
    near(state.rotation[0],std::cos(.35*time),1e-12,0);
    near(state.rotation[3],std::sin(.35*time),1e-12,0);
    for(unsigned axis=0;axis<3;++axis) {
      near(state.omega[axis],axis==2?.7:0,1e-12,0);
      near(state.alpha[axis],0,1e-12,0);
    }
  }
  Controls static_controls{};
  for(auto& control:static_controls) control={1,0,0,0,1,2,3};
  derivative_checks(static_controls,.43,interval);
  for(auto& control:static_controls) {
    control[0]=-1;
    control[1]=1e-14;
  }
  derivative_checks(static_controls,0,interval);
}
}

int main() {
  try {
    analytic_checks();
    std::mt19937_64 generator(2187);
    std::uniform_real_distribution<double> distribution(-1,1);
    for(unsigned trial=0;trial<30;++trial) {
      Controls controls;
      for(auto& control:controls) {
        const std::array<double,3> angle{.6*distribution(generator),.6*distribution(generator),.6*distribution(generator)};
        ceres::AngleAxisToQuaternion(angle.data(),control.data());
        const double scale=1.2+.3*distribution(generator);
        for(unsigned component=0;component<4;++component) control[component]*=scale;
        for(unsigned axis=0;axis<3;++axis) control[4+axis]=distribution(generator);
      }
      const double interval=.15+.2*(1+distribution(generator));
      for(const double phase:{0.,.37,1.}) derivative_checks(controls,phase,interval);
      const auto left=flatten(sample(controls,1,interval));
      const auto right=flatten(sample(controls,0,interval,1));
      for(unsigned component=0;component<left.size();++component) near(left[component],right[component],2e-12,2e-12);
      auto flipped=controls;
      for(unsigned component=0;component<4;++component) {
        flipped[1][component]*=-1;
        flipped[3][component]*=-1;
      }
      const auto original_values=flatten(sample(controls,.37,interval));
      const auto flipped_values=flatten(sample(flipped,.37,interval));
      for(unsigned component=0;component<original_values.size();++component) near(original_values[component],flipped_values[component],2e-12,2e-12);
    }
    Controls valid{};
    for(auto& control:valid) control[0]=1;
    for(const double interval:{0.,-1.}) {
      bool refused=false;
      try { static_cast<void>(sample(valid,.5,interval)); } catch(const std::invalid_argument&) { refused=true; }
      if(!refused) throw std::runtime_error("Invalid spline interval accepted");
    }
    valid[1][0]=0;
    bool refused=false;
    try { static_cast<void>(sample(valid,.5,1)); } catch(const std::invalid_argument&) { refused=true; }
    if(!refused) throw std::runtime_error("Zero quaternion accepted");
    std::cout<<"VI spline: analytic motion, 92 time/29-column derivative cases, C2 boundaries and quaternion signs passed\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';
    return 1;
  }
}
