#include "calib/vi.hpp"
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <optional>
#include <set>
#include <sys/wait.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace calib {
namespace {
void publish(const fs::path& path,const std::string& contents) {
  if(fs::exists(path) || fs::exists(path.string()+".incomplete")) throw Error(5,"VI output already exists: "+path.string());
  const auto temporary=fs::path(path.string()+".incomplete");
  std::ofstream out(temporary,std::ios::binary);
  out.exceptions(std::ios::badbit|std::ios::failbit);
  out<<contents;
  out.close();
  fs::rename(temporary,path);
}

double required_positive(const YAML::Node& root,const char* key) {
  const auto value=root[key];
  if(!value || !value.IsScalar()) throw Error(2,std::string("VI noise YAML requires scalar ")+key);
  try {
    const double parsed=value.as<double>();
    if(!std::isfinite(parsed) || parsed<=0) throw Error(2,std::string("VI noise YAML requires finite positive ")+key);
    return parsed;
  } catch(const YAML::Exception& error) {
    throw Error(2,std::string("Invalid VI noise YAML value for ")+key+": "+error.what());
  }
}

std::optional<double> optional_nonnegative(const YAML::Node& root,const char* key) {
  const auto value=root[key];
  if(!value) return std::nullopt;
  if(!value.IsScalar()) throw Error(2,std::string("VI noise YAML requires scalar ")+key);
  try {
    const double parsed=value.as<double>();
    if(!std::isfinite(parsed) || parsed<0) throw Error(2,std::string("VI noise YAML requires finite nonnegative ")+key);
    return parsed;
  } catch(const YAML::Exception& error) {
    throw Error(2,std::string("Invalid VI noise YAML value for ")+key+": "+error.what());
  }
}

void optional_update_rate(const YAML::Node& root) {
  const auto value=root["update_rate"];
  if(!value) return;
  if(!value.IsScalar()) throw Error(2,"VI noise YAML requires scalar update_rate");
  try {
    const double parsed=value.as<double>();
    if(!std::isfinite(parsed) || parsed<=0) throw Error(2,"VI noise YAML requires finite positive update_rate");
  } catch(const YAML::Exception& error) {
    throw Error(2,std::string("Invalid VI noise YAML value for update_rate: ")+error.what());
  }
}

void optional_topic(const YAML::Node& root) {
  const auto value=root["rostopic"];
  if(value && !value.IsScalar()) throw Error(2,"VI noise YAML requires scalar rostopic");
}

void matrix_yaml(YAML::Emitter& output,const TransformMatrix& matrix) {
  output<<YAML::BeginSeq;
  for(const auto& row:matrix) output<<YAML::Flow<<std::vector<double>(row.begin(),row.end());
  output<<YAML::EndSeq;
}

fs::path required_override(const char* name) {
  const char* value=std::getenv(name);
  if(!value || !*value) return {};
  const fs::path path=value;
  if(!fs::is_regular_file(path)) throw Error(5,std::string(name)+" does not name a regular file: "+path.string());
  return path;
}

fs::path executable_directory() {
  std::array<char,4096> buffer{};
  const auto length=readlink("/proc/self/exe",buffer.data(),buffer.size()-1);
  if(length<0 || static_cast<std::size_t>(length)>=buffer.size()-1) return {};
  return fs::path(std::string(buffer.data(),static_cast<std::size_t>(length))).parent_path();
}

fs::path vi_report_renderer() {
  if(const auto override=required_override("CALIB_VI_REPORT_SCRIPT");!override.empty()) return override;
  const auto executable=executable_directory();
  const std::array<fs::path,3> candidates{
    executable.empty()?fs::path{}:executable.parent_path()/CALIB_VI_REPORT_INSTALL_RELATIVE,
    fs::path(CALIB_VI_REPORT_SCRIPT),fs::path(CALIB_VI_REPORT_INSTALL_PATH)};
  for(const auto& candidate:candidates) if(!candidate.empty() && fs::is_regular_file(candidate)) return candidate;
  throw Error(5,"Cannot locate render_vi_report.py beside the installation, at the configured install prefix, or in the source tree");
}

fs::path vi_report_python() {
  if(const auto override=required_override("CALIB_PYTHON_EXECUTABLE");!override.empty()) return override;
  for(const fs::path& candidate:{fs::path(CALIB_PYTHON_EXECUTABLE),fs::path("/usr/bin/python3"),fs::path("/usr/local/bin/python3")})
    if(fs::is_regular_file(candidate)) return candidate;
  throw Error(5,"Cannot locate a Python interpreter for the VI report; set CALIB_PYTHON_EXECUTABLE");
}

}

ViNoise read_vi_noise(const fs::path& path) {
  YAML::Node root;
  try {
    root=YAML::LoadFile(path.string());
  } catch(const YAML::Exception& error) {
    throw Error(2,std::string("Cannot read VI noise YAML: ")+error.what());
  }
  ViNoise result;
  if(!root.IsMap()) throw Error(2,"VI noise YAML must be a mapping");
  std::set<std::string> keys;
  for(const auto& entry:root) {
    const auto key=entry.first.as<std::string>();
    if(!keys.insert(key).second) throw Error(2,"Duplicate VI noise YAML key: "+key);
  }
  result.gyroscope_noise_density=required_positive(root,"gyroscope_noise_density");
  result.accelerometer_noise_density=required_positive(root,"accelerometer_noise_density");
  const double common_random_walk=optional_nonnegative(root,"random_walk").value_or(0.0);
  result.gyroscope_random_walk=optional_nonnegative(root,"gyroscope_random_walk").value_or(common_random_walk);
  result.accelerometer_random_walk=optional_nonnegative(root,"accelerometer_random_walk").value_or(common_random_walk);
  optional_update_rate(root);
  optional_topic(root);
  return result;
}

Json vi_json(const ViCalibration& result) {
  return Json{
    {"schema_version",2},
    {"status",result.diagnostics.is_object()?result.diagnostics.value("status", "converged"):"converged"},
    {"camera",{{"camera",result.camera.id},{"camera_model",result.camera.model},
      {"distortion_model",result.camera.distortion},{"intrinsics",result.camera.intrinsics},
      {"distortion_coeffs",result.camera.coefficients},{"resolution",{result.camera.width,result.camera.height}},
      {"line_delay",0.0}}},
    {"T_cam_imu",result.T_cam_imu},
    {"timeshift_cam_imu",result.timeshift_cam_imu},
    {"gyro_bias",result.gyro_bias},
    {"accel_bias",result.accel_bias},
    {"gravity_world",result.gravity_world},
    {"conventions",{{"transform","T_cam_imu transforms IMU coordinates into camera coordinates"},
      {"time","t_imu = t_cam + timeshift_cam_imu"},{"translation_units","m"},
      {"time_units","s"},{"angular_units","rad"},{"acceleration_units","m/s^2"},
      {"world_frame","AprilGrid/mire frame"},{"bias_model",result.diagnostics.value("bias_model","unknown")},
      {"bias_summary","gyro_bias and accel_bias are the first bias control; use bias_trajectory for the complete model"},
      {"covariance_perturbation","left rotation perturbation, additive translation and timeshift"}}},
    {"covariance",result.diagnostics.is_object()?result.diagnostics.value("covariance",Json(nullptr)):Json(nullptr)},
    {"quality","provisional_not_certified"},
    {"diagnostics",result.diagnostics},
    {"trajectory",result.trajectory},
    {"bias_trajectory",result.bias_trajectory}
  };
}

void write_vi_camchain(const fs::path& path,const ViCalibration& result) {
  YAML::Emitter output;
  output.SetDoublePrecision(17);
  output<<YAML::BeginMap<<YAML::Key<<result.camera.id<<YAML::Value<<YAML::BeginMap
    <<YAML::Key<<"camera_model"<<YAML::Value<<result.camera.model
    <<YAML::Key<<"distortion_model"<<YAML::Value<<result.camera.distortion
    <<YAML::Key<<"intrinsics"<<YAML::Value<<YAML::Flow<<result.camera.intrinsics
    <<YAML::Key<<"distortion_coeffs"<<YAML::Value<<YAML::Flow<<result.camera.coefficients
    <<YAML::Key<<"resolution"<<YAML::Value<<YAML::Flow<<std::vector<std::uint32_t>{result.camera.width,result.camera.height}
    <<YAML::Key<<"T_cam_imu"<<YAML::Value;
  matrix_yaml(output,result.T_cam_imu);
  output<<YAML::Key<<"timeshift_cam_imu"<<YAML::Value<<result.timeshift_cam_imu
    <<YAML::Key<<"line_delay"<<YAML::Value<<0.0
    <<YAML::EndMap<<YAML::EndMap;
  if(!output.good()) throw Error(5,"Cannot serialize VI camchain");
  publish(path,"# VI global-shutter estimate. T_cam_imu maps IMU coordinates into camera coordinates.\n# Bias estimates and their temporal model are in calibration.json.\n"+std::string(output.c_str())+'\n');
}

void write_vi_pdf(const fs::path& path,const ViCalibration& result) {
  const auto temporary=fs::path(path.string()+".incomplete");
  const auto data_path=fs::path(path.string()+".data.incomplete");
  if(fs::exists(path) || fs::exists(temporary) || fs::exists(data_path))
    throw Error(5,"VI report output already exists: "+path.string());

  Json motion=Json::array();
  for(const auto& sample:result.report.motion)
    motion.push_back({{"time_s",sample.time_s},{"position_world_m",sample.position_world_m},
      {"rotation_world_camera_wxyz",sample.rotation_world_camera_wxyz}});
  Json reprojection=Json::array();
  for(const auto& frame:result.report.reprojection)
    reprojection.push_back({{"timestamp_ns",frame.timestamp_ns},{"time_s",frame.time_s},
      {"held_out",frame.held_out},{"residuals_px",frame.residuals_px}});
  Json imu=Json::array();
  for(const auto& sample:result.report.imu)
    imu.push_back({{"timestamp_ns",sample.timestamp_ns},{"time_s",sample.time_s},
      {"measured_gyro_rad_s",sample.measured_gyro_rad_s},{"predicted_gyro_rad_s",sample.predicted_gyro_rad_s},
      {"gyro_residual_rad_s",sample.gyro_residual_rad_s},{"measured_accel_m_s2",sample.measured_accel_m_s2},
      {"predicted_accel_m_s2",sample.predicted_accel_m_s2},{"accel_residual_m_s2",sample.accel_residual_m_s2},
      {"gyro_bias_rad_s",sample.gyro_bias_rad_s},{"accel_bias_m_s2",sample.accel_bias_m_s2}});
  const Json report{
    {"schema_version",3},
    {"camera_id",result.camera.id},
    {"camera",{{"camera_model",result.camera.model},{"distortion_model",result.camera.distortion},
      {"intrinsics",result.camera.intrinsics},{"distortion_coeffs",result.camera.coefficients},
      {"resolution",{result.camera.width,result.camera.height}},{"line_delay_s",0.0}}},
    {"T_cam_imu",result.T_cam_imu},
    {"timeshift_cam_imu",result.timeshift_cam_imu},
    {"gyro_bias",result.gyro_bias},
    {"accel_bias",result.accel_bias},
    {"gravity_world",result.gravity_world},
    {"diagnostics",result.diagnostics},
    {"motion",std::move(motion)},
    {"reprojection",std::move(reprojection)},
    {"imu",std::move(imu)}
  };
  {
    std::ofstream stream(data_path);
    stream.exceptions(std::ios::badbit|std::ios::failbit);
    stream<<report;
    stream.close();
  }

  const std::string python=vi_report_python().string();
  const std::string renderer=vi_report_renderer().string();
  const std::string input=data_path.string(),output=temporary.string();
  const pid_t child=fork();
  if(child<0) throw Error(5,"Cannot start VI report renderer: "+std::string(std::strerror(errno)));
  if(child==0) {
    execl(python.c_str(),python.c_str(),renderer.c_str(),input.c_str(),output.c_str(),
      static_cast<char*>(nullptr));
    _exit(127);
  }
  int status{};
  pid_t waited;
  do waited=waitpid(child,&status,0); while(waited<0 && errno==EINTR);
  if(waited<0) throw Error(5,"Cannot wait for VI report renderer: "+std::string(std::strerror(errno)));
  if(!WIFEXITED(status) || WEXITSTATUS(status)!=0) {
    const std::string detail=WIFEXITED(status)?"exit "+std::to_string(WEXITSTATUS(status)):
      WIFSIGNALED(status)?"signal "+std::to_string(WTERMSIG(status)):"unknown status";
    throw Error(5,"VI report renderer failed ("+detail+")");
  }
  std::ifstream generated(temporary,std::ios::binary);
  char signature[4]{};
  generated.read(signature,4);
  if(generated.gcount()!=4 || std::string_view(signature,4)!="%PDF")
    throw Error(5,"VI report renderer did not produce a valid PDF");
  generated.close();
  fs::rename(temporary,path);
  std::error_code ignored;
  fs::remove(data_path,ignored);
}
}
