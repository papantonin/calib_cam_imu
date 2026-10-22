#pragma once
#include "calib/rig.hpp"

namespace calib {
struct ViNoise {
  double gyroscope_noise_density{},accelerometer_noise_density{};
  double gyroscope_random_walk{},accelerometer_random_walk{};
};
struct ViOptions {
  ViNoise noise;
  double knot_interval_s{0.05},bias_knot_interval_s{0.1},max_offset_s{0.05},pixel_sigma{0.8},gravity_m_s2{9.80665},bootstrap_block_s{};
  unsigned threads{1},max_iterations{80},bootstrap_replicates{},bootstrap_max_iterations{40};
  std::uint64_t bootstrap_seed{20261020};
  std::function<bool()> cancelled;
};
struct ViMotionSample {
  double time_s{};
  std::array<double,3> position_world_m{};
  std::array<double,4> rotation_world_camera_wxyz{};
};
struct ViReprojectionFrame {
  std::int64_t timestamp_ns{};
  double time_s{};
  bool held_out{};
  std::vector<std::array<double,2>> residuals_px;
};
struct ViImuReportSample {
  std::int64_t timestamp_ns{};
  double time_s{};
  std::array<double,3> measured_gyro_rad_s{},predicted_gyro_rad_s{},gyro_residual_rad_s{};
  std::array<double,3> measured_accel_m_s2{},predicted_accel_m_s2{},accel_residual_m_s2{};
  std::array<double,3> gyro_bias_rad_s{},accel_bias_m_s2{};
};
struct ViReportData {
  std::vector<ViMotionSample> motion;
  std::vector<ViReprojectionFrame> reprojection;
  std::vector<ViImuReportSample> imu;
};
struct ViCalibration {
  CameraCalibration camera;
  TransformMatrix T_cam_imu;
  double timeshift_cam_imu{};
  std::array<double,3> gyro_bias{},accel_bias{},gravity_world{};
  Json diagnostics,trajectory,bias_trajectory,bootstrap;
  ViReportData report;
};
ViNoise read_vi_noise(const fs::path& path);
ViCalibration calibrate_vi(const AprilGridGeometry& geometry,const CameraCalibration& camera,
    std::span<const FrameObservations> frames,std::span<const ImuSample> imu,const ViOptions& options);
Json vi_json(const ViCalibration& result);
void write_vi_camchain(const fs::path& path,const ViCalibration& result);
void write_vi_pdf(const fs::path& path,const ViCalibration& result);
}
