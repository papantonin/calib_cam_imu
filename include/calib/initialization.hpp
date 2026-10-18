#pragma once
#include "calib/observations.hpp"
#include <functional>
#include <optional>

namespace calib {
// Bounded deterministic selection: strongest eligible view in each temporal bin.
class InitialViewSampler {
 public:
  InitialViewSampler(std::size_t frames,std::size_t max_views);
  void consider(FrameObservations frame);
  std::vector<FrameObservations> selected() const;
 private:
  std::size_t frames_,seen_{};
  std::vector<std::optional<FrameObservations>> bins_;
};
struct InitialPose {
  std::int64_t timestamp_ns{};
  std::array<std::array<double,4>,4> T_cam_target{};
  double rms_px{};
  std::size_t corners{};
  bool held_out{};
};
struct InitializationRejection {
  std::int64_t timestamp_ns{};
  std::uint32_t point_id{};
  std::string reason;
  double residual_px{};
};
struct InitialCalibration {
  CameraCalibration camera;
  std::vector<InitialPose> poses;
  std::vector<InitializationRejection> rejected;
  std::vector<std::string> failed_attempts;
  double training_rms_px{},held_out_rms_px{},normal_spread_rad{};
  double max_observed_angle_rad{},min_sampled_distortion_eigenvalue{};
  double max_inverse_roundtrip_ray_error{};
  std::size_t selected_views{};
};
// OpenCV adapter. No image storage, YAML, or cross-camera/IMU solve here.
InitialCalibration initialize_camera(const AprilGridGeometry& geometry,
    std::span<const FrameObservations> frames,DistortionModel distortion,
    const std::function<bool()>& cancelled={});
Json initialization_json(const InitialCalibration& result);
InitialPose estimate_target_pose(const AprilGridGeometry& geometry,
    const FrameObservations& frame,const CameraCalibration& camera);
struct OpticalFieldCheck { double minimum_eigenvalue{},inverse_ray_error{}; };
OpticalFieldCheck validate_optical_field(const CameraCalibration& camera,double max_normalized_radius);
void write_initial_camchain(const fs::path& path,std::span<const InitialCalibration> cameras);
} // namespace calib
