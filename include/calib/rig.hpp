#pragma once
#include "calib/initialization.hpp"
#include <map>

namespace calib {
using TransformMatrix=std::array<std::array<double,4>,4>;
struct SynchronizedView {
  std::int64_t timestamp_ns{};
  bool held_out{};
  std::vector<FrameObservations> cameras;
};
class RigViewSampler {
 public:
  RigViewSampler(const Dataset& dataset,std::size_t max_views);
  void consider(const FrameObservations& frame);
  bool held_out(std::int64_t timestamp) const;
  std::vector<SynchronizedView> selected() const;
 private:
  std::map<std::int64_t,SynchronizedView> views_;
};
struct RigOptions { unsigned threads{1}; std::function<bool()> cancelled; };
struct RigCamera {
  CameraCalibration optical;
  TransformMatrix T_cam_reference;
  std::array<double,3> center_in_reference_m;
  double training_rms_px{},held_out_rms_px{};
  std::size_t training_corners{},held_out_corners{};
  OpticalFieldCheck optical_check;
};
struct RigCalibration {
  std::string reference_camera;
  std::vector<RigCamera> cameras;
  Json diagnostics;
};
RigCalibration calibrate_rig(const AprilGridGeometry& geometry,std::span<const InitialCalibration> initial,
    std::span<const SynchronizedView> views,const RigOptions& options={});
Json rig_json(const RigCalibration& result);
void write_rig_camchain(const fs::path& path,const RigCalibration& result);
void write_rig_pdf(const fs::path& path,const RigCalibration& result);
} // namespace calib
