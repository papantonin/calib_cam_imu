#pragma once
#include "calib/vi_spline.hpp"
#include "calib/camera_model.hpp"
#include <algorithm>
#include <cmath>

namespace calib::vi_detail {
template<class T> std::array<T,3> cross_product(const std::array<T,3>& first,const std::array<T,3>& second) {
  return {first[1]*second[2]-first[2]*second[1],first[2]*second[0]-first[0]*second[2],first[0]*second[1]-first[1]*second[0]};
}
template<class T> std::array<T,3> inverse_rotate(const T* rotation,const std::array<T,3>& vector) {
  const T inverse[4]={rotation[0],-rotation[1],-rotation[2],-rotation[3]};
  std::array<T,3> result;
  ceres::QuaternionRotatePoint(inverse,vector.data(),result.data());
  return result;
}
struct VisualResidual {
  std::array<double,3> point;
  std::array<double,2> pixel;
  CameraModel::Parameters optical;
  DistortionModel distortion;
  double coordinate{},interval{},sigma{};
  template<class T> bool operator()(T const* const* blocks,T* residual) const {
    const auto state=evaluate(blocks,T(coordinate),interval);
    std::array<T,3> source;
    for(unsigned axis=0;axis<3;++axis) source[axis]=T(point[axis])-state.position[axis];
    const auto sensor=inverse_rotate(state.rotation.data(),source);
    if(sensor[2]<=T(1e-6)) return false;
    const T horizontal=sensor[0]/sensor[2],vertical=sensor[1]/sensor[2];
    const T radius2=horizontal*horizontal+vertical*vertical;
    T distorted_x,distorted_y;
    if(distortion==DistortionModel::radtan) {
      const T scale=T(1)+T(optical[4])*radius2+T(optical[5])*radius2*radius2;
      distorted_x=horizontal*scale+T(2*optical[6])*horizontal*vertical+T(optical[7])*(radius2+T(2)*horizontal*horizontal);
      distorted_y=vertical*scale+T(optical[6])*(radius2+T(2)*vertical*vertical)+T(2*optical[7])*horizontal*vertical;
    } else {
      T angle_scale;
      if(radius2<T(1e-8)) angle_scale=T(1)+radius2*(T(-1.0/3)+radius2/T(5));
      else { const T radius=sqrt(radius2); angle_scale=atan(radius)/radius; }
      const T angle2=radius2*angle_scale*angle_scale;
      const T scale=angle_scale*(T(1)+angle2*(T(optical[4])+angle2*(T(optical[5])+angle2*(T(optical[6])+angle2*T(optical[7])))));
      distorted_x=horizontal*scale; distorted_y=vertical*scale;
    }
    residual[0]=(T(optical[0])*distorted_x+T(optical[2]-pixel[0]))/T(sigma);
    residual[1]=(T(optical[1])*distorted_y+T(optical[3]-pixel[1]))/T(sigma);
    return true;
  }
};
struct ImuResidual {
  std::array<double,6> measurement;
  double time{},interval{},gyro_sigma{},accel_sigma{};
  int first_segment{},segment_count{};
  double bias_fraction{};
  bool variable_bias{};
  template<class T> bool operator()(T const* const* blocks,T* residual) const {
    const T coordinate=(T(time)-blocks[3][0])/T(interval);
    const int segment=static_cast<int>(std::floor(scalar_part(coordinate)));
    if(segment<first_segment || segment>=first_segment+segment_count) return false;
    const int trajectory_block=variable_bias?5:4;
    const auto state=evaluate(blocks+trajectory_block+segment-first_segment,coordinate-T(segment),interval);
    const std::array<T,3> lever{blocks[0][4],blocks[0][5],blocks[0][6]};
    auto force=inverse_rotate(state.rotation.data(),std::array<T,3>{state.acceleration[0]-blocks[2][0],state.acceleration[1]-blocks[2][1],state.acceleration[2]-blocks[2][2]});
    const auto tangential=cross_product(state.alpha,lever);
    const auto centripetal=cross_product(state.omega,cross_product(state.omega,lever));
    for(unsigned axis=0;axis<3;++axis) force[axis]+=tangential[axis]+centripetal[axis];
    const auto gyro=inverse_rotate(blocks[0],state.omega);
    const auto accel=inverse_rotate(blocks[0],force);
    for(unsigned axis=0;axis<3;++axis) {
      const T bias=variable_bias?(T(1-bias_fraction)*blocks[1][axis]+T(bias_fraction)*blocks[4][axis]):blocks[1][axis];
      const T accel_bias=variable_bias?(T(1-bias_fraction)*blocks[1][axis+3]+T(bias_fraction)*blocks[4][axis+3]):blocks[1][axis+3];
      residual[axis]=(gyro[axis]+bias-T(measurement[axis]))/T(gyro_sigma);
      residual[axis+3]=(accel[axis]+accel_bias-T(measurement[axis+3]))/T(accel_sigma);
    }
    return true;
  }
};
struct BiasRandomWalkResidual {
  double gyro_sigma{},accel_sigma{};
  template<class T> bool operator()(const T* previous,const T* next,T* residual) const {
    for(unsigned axis=0;axis<3;++axis) {
      residual[axis]=(next[axis]-previous[axis])/T(gyro_sigma);
      residual[axis+3]=(next[axis+3]-previous[axis+3])/T(accel_sigma);
    }
    return true;
  }
  template<class T> bool operator()(T const* const* blocks,T* residual) const {
    return (*this)(blocks[0],blocks[1],residual);
  }
};
}
