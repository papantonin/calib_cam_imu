#include "calib/initialization.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace calib {
Json initialization_json(const InitialCalibration& r) {
  Json out{{"schema_version",1},{"status","initial_estimate"},{"camera",r.camera.id},
    {"backend",std::string("OpenCV ")+CALIB_OPENCV_VERSION},{"quality","not_certified"},
    {"camera_model",r.camera.model},{"distortion_model",r.camera.distortion},
    {"intrinsics",r.camera.intrinsics},{"distortion_coeffs",r.camera.coefficients},
    {"resolution",{r.camera.width,r.camera.height}},{"selected_views",r.selected_views},
    {"training_rms_px",r.training_rms_px},{"held_out_rms_px",r.held_out_rms_px},
    {"normal_spread_rad",r.normal_spread_rad},{"max_observed_angle_rad",r.max_observed_angle_rad},
    {"distortion_check",{{"kind","sampled_local_invertibility_over_observed_radius"},
      {"radial_samples",65},{"azimuth_samples",32},{"minimum_eigenvalue",r.min_sampled_distortion_eigenvalue},
      {"max_inverse_roundtrip_ray_error",r.max_inverse_roundtrip_ray_error},
      {"global_injectivity_proven",false}}},
    {"covariance",nullptr},{"poses",Json::array()},{"rejected_points",Json::array()},
    {"failed_attempts",r.failed_attempts},
    {"conventions",{{"pose","p_cam = T_cam_target * p_target"},{"translation_units","m"},
      {"shutter","global_shutter_assumption; no readout or offset estimated"},
      {"held_out","intrinsics fixed; pose fitted on that view; all its corners scored"}}},
    {"policy_version",1}};
  for(const auto& pose:r.poses) out["poses"].push_back({{"timestamp_ns",pose.timestamp_ns},
    {"T_cam_target",pose.T_cam_target},{"rms_px",pose.rms_px},{"corners",pose.corners},{"held_out",pose.held_out}});
  for(const auto& reject:r.rejected) out["rejected_points"].push_back({{"timestamp_ns",reject.timestamp_ns},
    {"point_id",reject.point_id},{"reason",reject.reason},{"initial_residual_px",reject.residual_px}});
  return out;
}
void write_initial_camchain(const fs::path& path,std::span<const InitialCalibration> cameras) {
  if(fs::exists(path) || fs::exists(path.string()+".incomplete")) throw Error(5,"Initial camchain output already exists");
  YAML::Emitter out; out.SetDoublePrecision(17); out<<YAML::BeginMap;
  for(const auto& result:cameras) {
    const auto& c=result.camera;
    out<<YAML::Key<<c.id<<YAML::Value<<YAML::BeginMap
      <<YAML::Key<<"camera_model"<<YAML::Value<<c.model
      <<YAML::Key<<"distortion_model"<<YAML::Value<<c.distortion
      <<YAML::Key<<"intrinsics"<<YAML::Value<<YAML::Flow<<c.intrinsics
      <<YAML::Key<<"distortion_coeffs"<<YAML::Value<<YAML::Flow<<c.coefficients
      <<YAML::Key<<"resolution"<<YAML::Value<<YAML::Flow<<std::vector<std::uint32_t>{c.width,c.height}
      <<YAML::EndMap;
  }
  out<<YAML::EndMap;
  if(!out.good()) throw Error(5,"Cannot serialize initial camchain");
  const fs::path temporary=path.string()+".incomplete";
  std::ofstream stream(temporary); stream.exceptions(std::ios::failbit|std::ios::badbit);
  stream<<"# Initial optical estimates only; see initialization.json. No rig extrinsics or shutter calibration.\n"<<out.c_str()<<'\n';
  stream.close(); fs::rename(temporary,path);
}
} // namespace calib
