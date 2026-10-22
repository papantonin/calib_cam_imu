#pragma once
#include <ceres/jet.h>
#include <ceres/rotation.h>
#include <array>
#include <cmath>
#include <stdexcept>

namespace calib::vi_detail {
template<class T> double scalar_part(const T& value) { return static_cast<double>(value); }
template<class T,int Size> double scalar_part(const ceres::Jet<T,Size>& value) { return scalar_part(value.a); }

template<class T> struct SplineState {
  std::array<T,4> rotation;
  std::array<T,3> position,acceleration,omega,alpha;
};

template<class T> std::array<T,4> conjugate(const std::array<T,4>& quaternion) {
  return {quaternion[0],-quaternion[1],-quaternion[2],-quaternion[3]};
}

template<class T> std::array<T,3> cross(const std::array<T,3>& left,const std::array<T,3>& right) {
  return {left[1]*right[2]-left[2]*right[1],left[2]*right[0]-left[0]*right[2],left[0]*right[1]-left[1]*right[0]};
}

template<class T> SplineState<T> evaluate(const T* const* knots,const T& phase,double interval) {
  using std::sqrt;
  if(!std::isfinite(interval) || interval<=0) throw std::invalid_argument("Spline interval must be finite and positive");
  const T phase2=phase*phase,phase3=phase2*phase;
  const T complement=T(1)-phase;
  const std::array<T,4> basis{complement*complement*complement/T(6),
    (T(3)*phase3-T(6)*phase2+T(4))/T(6),
    (-T(3)*phase3+T(3)*phase2+T(3)*phase+T(1))/T(6),phase3/T(6)};
  const std::array<T,4> second{complement,T(3)*phase-T(2),T(1)-T(3)*phase,phase};
  const std::array<T,3> cumulative{T(1)-basis[0],basis[2]+basis[3],basis[3]};
  const std::array<T,3> first{complement*complement/T(2),T(0.5)+phase-phase2,phase2/T(2)};
  const std::array<T,3> curvature{phase-T(1),T(1)-T(2)*phase,phase};
  const T inverse_interval=T(1)/T(interval),inverse_interval2=inverse_interval*inverse_interval;
  std::array<std::array<T,4>,4> rotations;
  SplineState<T> state{};
  for(unsigned control=0;control<4;++control) {
    T squared_norm=T(0);
    for(unsigned component=0;component<4;++component) squared_norm+=knots[control][component]*knots[control][component];
    if(!(scalar_part(squared_norm)>0) || !std::isfinite(scalar_part(squared_norm)))
      throw std::invalid_argument("Spline quaternion must have finite nonzero norm");
    const T inverse_norm=T(1)/sqrt(squared_norm);
    for(unsigned component=0;component<4;++component) rotations[control][component]=knots[control][component]*inverse_norm;
    for(unsigned axis=0;axis<3;++axis) {
      state.position[axis]+=basis[control]*knots[control][4+axis];
      state.acceleration[axis]+=second[control]*inverse_interval2*knots[control][4+axis];
    }
  }
  state.rotation=rotations[0];
  for(unsigned increment=0;increment<3;++increment) {
    const auto inverse_previous=conjugate(rotations[increment]);
    std::array<T,4> relative,exponential,updated_rotation;
    std::array<T,3> tangent,scaled_tangent,local_omega,rotated_omega,rotated_alpha;
    ceres::QuaternionProduct(inverse_previous.data(),rotations[increment+1].data(),relative.data());
    ceres::QuaternionToAngleAxis(relative.data(),tangent.data());
    for(unsigned axis=0;axis<3;++axis) {
      scaled_tangent[axis]=cumulative[increment]*tangent[axis];
      local_omega[axis]=first[increment]*inverse_interval*tangent[axis];
    }
    ceres::AngleAxisToQuaternion(scaled_tangent.data(),exponential.data());
    const auto inverse_exponential=conjugate(exponential);
    ceres::UnitQuaternionRotatePoint(inverse_exponential.data(),state.omega.data(),rotated_omega.data());
    ceres::UnitQuaternionRotatePoint(inverse_exponential.data(),state.alpha.data(),rotated_alpha.data());
    const auto transport=cross(local_omega,rotated_omega);
    for(unsigned axis=0;axis<3;++axis) {
      state.omega[axis]=rotated_omega[axis]+local_omega[axis];
      state.alpha[axis]=rotated_alpha[axis]-transport[axis]+curvature[increment]*inverse_interval2*tangent[axis];
    }
    ceres::QuaternionProduct(state.rotation.data(),exponential.data(),updated_rotation.data());
    state.rotation=updated_rotation;
  }
  return state;
}
}
