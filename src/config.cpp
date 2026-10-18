#include "calib/core.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <yaml-cpp/yaml.h>

namespace calib {
namespace {
void check_yaml(const YAML::Node& node,int depth=0) {
  if (depth>32) throw Error(2,"YAML nesting/alias depth exceeds 32");
  if (node.IsMap()) {
    std::set<std::string> seen;
    for (const auto& pair:node) {
      if (!pair.first.IsScalar()) throw Error(2,"YAML keys must be scalar");
      auto key=pair.first.as<std::string>();
      if (!seen.insert(key).second) throw Error(2,"Duplicate YAML key: "+key);
      check_yaml(pair.second,depth+1);
    }
  } else if (node.IsSequence()) for (const auto& child:node) check_yaml(child,depth+1);
}
YAML::Node load(const fs::path& path) {
  if (!fs::is_regular_file(path)) throw Error(5,"Cannot read YAML: "+path.string());
  if (fs::file_size(path)>1024*1024) throw Error(2,"Configuration exceeds 1 MiB: "+path.string());
  auto root=YAML::LoadFile(path.string());
  check_yaml(root);
  if (!root.IsMap()) throw Error(2,"YAML root must be a map: "+path.string());
  return root;
}
void finite(double x,std::string_view key) {
  if (!std::isfinite(x)) throw Error(2,"Non-finite YAML value: "+std::string(key));
}
}
CameraModel make_camera_model(const CameraCalibration& camera) {
  if(camera.model!="pinhole" || (camera.distortion!="radtan" && camera.distortion!="equidistant"))
    throw Error(8,"Unsupported optical model: "+camera.id);
  if(camera.intrinsics.size()!=4 || camera.coefficients.size()!=4)
    throw Error(2,"Expected four intrinsic and four distortion parameters: "+camera.id);
  CameraModel::Parameters parameters;
  std::copy(camera.intrinsics.begin(),camera.intrinsics.end(),parameters.begin());
  std::copy(camera.coefficients.begin(),camera.coefficients.end(),parameters.begin()+4);
  try {
    return CameraModel(camera.distortion=="radtan" ? DistortionModel::radtan : DistortionModel::equidistant,parameters);
  } catch(const std::invalid_argument& error) { throw Error(2,camera.id+": "+error.what()); }
}
std::vector<CameraCalibration> read_camchain(const fs::path& path) {
  try {
    auto root=load(path);
    std::vector<CameraCalibration> cameras;
    for (const auto& pair:root) {
      CameraCalibration camera;
      camera.id=pair.first.as<std::string>();
      if (!camera.id.starts_with("cam")) throw Error(2,"Unexpected camchain entry: "+camera.id);
      const auto suffix=std::string_view(camera.id).substr(3);
      (void)parse_timestamp(suffix,camera.id);
      if (suffix.size()>1 && suffix.front()=='0') throw Error(2,"Noncanonical camera id: "+camera.id);
      auto node=pair.second;
      camera.model=node["camera_model"].as<std::string>();
      camera.distortion=node["distortion_model"].as<std::string>();
      if (camera.model!="pinhole" || (camera.distortion!="radtan" && camera.distortion!="equidistant"))
        throw Error(8,"Camchain inspection currently supports pinhole/radtan and pinhole/equidistant");
      camera.intrinsics=node["intrinsics"].as<std::vector<double>>();
      camera.coefficients=node["distortion_coeffs"].as<std::vector<double>>();
      (void)make_camera_model(camera);
      auto size=node["resolution"].as<std::vector<std::uint32_t>>();
      if(size.size()!=2 || !size[0] || !size[1] || size[0]>32768 || size[1]>32768) throw Error(2,"Invalid camera resolution");
      camera.width=size[0]; camera.height=size[1];
      auto delay=node["line_delay"], alias=node["lineDelay"];
      camera.has_line_delay=bool(delay)||bool(alias);
      if(camera.has_line_delay) {
        camera.line_delay_s=(delay?delay:alias).as<double>();
        finite(camera.line_delay_s,"line_delay");
        if(delay && alias && alias.as<double>()!=camera.line_delay_s) throw Error(2,"Conflicting line_delay and lineDelay");
      }
      if(node["rolling_shutter"]) throw Error(8,"Extended shutter conventions not yet implemented; only Kalibr row-0 vertical convention is supported");
      cameras.push_back(std::move(camera));
    }
    if(cameras.empty()) throw Error(2,"Empty camchain");
    return cameras;
  } catch(const YAML::Exception& e) { throw Error(2,path.string()+": "+e.what()); }
}
double corner_time(std::int64_t timestamp,std::int64_t origin,double row,double line_delay_s,double offset_s) {
  finite(row,"row"); finite(line_delay_s,"line_delay"); finite(offset_s,"offset");
  auto time=relative_seconds(timestamp,origin)+offset_s+row*line_delay_s;
  finite(time,"corner time");
  return time;
}
Target read_target(const fs::path& path) {
  try {
    const auto root=load(path);
    const std::set<std::string> keys{"target_type","tagRows","tagCols","tagSize","tagSpacing","tagFamily","tagStartId","tagBorderBits"};
    for(const auto& pair:root) if(!keys.contains(pair.first.as<std::string>())) throw Error(2,"Unknown target key: "+pair.first.as<std::string>());
    if(root["target_type"].as<std::string>()!="aprilgrid") throw Error(8,"Only AprilGrid targets supported");
    if(!root["tagFamily"] || !root["tagBorderBits"]) throw Error(2,"Explicit tagFamily and tagBorderBits required; legacy Kalibr grids cannot be assumed compatible");
    Target target;
    target.family=root["tagFamily"].as<std::string>();
    target.border_bits=root["tagBorderBits"].as<int>();
    if(target.border_bits!=1 && !(target.border_bits==2 && target.family=="tag36h11"))
      throw Error(8,"Supported borders: native family (1), or tag36h11 legacy border (2)");
    if(target.family!="tag36h11" && target.family!="tagStandard41h12") throw Error(8,"Unsupported tag family: "+target.family);
    target.rows=root["tagRows"].as<int>(); target.cols=root["tagCols"].as<int>();
    target.first_id=root["tagStartId"]?root["tagStartId"].as<int>():0;
    target.size_m=root["tagSize"].as<double>(); target.spacing_ratio=root["tagSpacing"].as<double>();
    finite(target.size_m,"tagSize"); finite(target.spacing_ratio,"tagSpacing");
    if(target.rows<1 || target.cols<1 || target.rows>1000 || target.cols>1000 || target.first_id<0 || target.first_id>100000 || target.size_m<=0 || target.spacing_ratio<0)
      throw Error(2,"Invalid AprilGrid dimensions/geometry/IDs");
    return target;
  } catch(const YAML::Exception& e) { throw Error(2,path.string()+": "+e.what()); }
}
} // namespace calib
