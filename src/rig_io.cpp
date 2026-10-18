#include "calib/rig.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace calib {
namespace {
TransformMatrix relative(const TransformMatrix& a,const TransformMatrix& b) {
  // a * inverse(b): maps the preceding camera into the current camera.
  TransformMatrix out{}; out[3][3]=1;
  for(unsigned r=0;r<3;++r) {
    for(unsigned c=0;c<3;++c) for(unsigned k=0;k<3;++k) out[r][c]+=a[r][k]*b[c][k];
    out[r][3]=a[r][3];
    for(unsigned k=0;k<3;++k) out[r][3]-=out[r][k]*b[k][3];
  }
  return out;
}
void publish(const fs::path& path,const std::string& contents) {
  if(fs::exists(path) || fs::exists(path.string()+".incomplete")) throw Error(5,"Rig output already exists: "+path.string());
  const auto temporary=fs::path(path.string()+".incomplete");
  std::ofstream out(temporary,std::ios::binary); out.exceptions(std::ios::badbit|std::ios::failbit);
  out<<contents; out.close(); fs::rename(temporary,path);
}
std::string number(double value) { std::ostringstream s; s<<std::fixed<<std::setprecision(6)<<value; return s.str(); }
std::string escaped(const std::string& text) {
  std::string out;
  for(char c:text) {
    if(c=='(' || c==')' || c=='\\') out+='\\';
    out+=(static_cast<unsigned char>(c)>=32 && static_cast<unsigned char>(c)<127)?c:'?';
  }
  return out;
}
void text(std::ostringstream& page,double x,double y,const std::string& value,double size=11) {
  page<<"0 0 0 rg BT /F1 "<<size<<" Tf "<<x<<' '<<y<<" Td ("<<escaped(value)<<") Tj ET\n";
}
}
Json rig_json(const RigCalibration& result) {
  Json out{{"schema_version",1},{"status","converged"},{"reference_camera",result.reference_camera},
    {"conventions",{{"transform","p_cam = T_cam_reference * p_reference"},
      {"translation_units","m"},{"camera_center","position of optical origin in reference camera frame"}}},
    {"cameras",Json::array()},{"diagnostics",result.diagnostics}};
  for(const auto& camera:result.cameras) out["cameras"].push_back({{"camera",camera.optical.id},
    {"camera_model",camera.optical.model},{"distortion_model",camera.optical.distortion},
    {"intrinsics",camera.optical.intrinsics},{"distortion_coeffs",camera.optical.coefficients},
    {"resolution",{camera.optical.width,camera.optical.height}},{"T_cam_reference",camera.T_cam_reference},
    {"camera_center_in_reference_m",camera.center_in_reference_m},
    {"training_rms_px",camera.training_rms_px},{"held_out_rms_px",camera.held_out_rms_px},
    {"training_corners",camera.training_corners},{"held_out_corners",camera.held_out_corners},
    {"optical_check",{{"minimum_sampled_eigenvalue",camera.optical_check.minimum_eigenvalue},
      {"inverse_roundtrip_ray_error",camera.optical_check.inverse_ray_error}}}});
  return out;
}
void write_rig_camchain(const fs::path& path,const RigCalibration& result) {
  YAML::Emitter out; out.SetDoublePrecision(17); out<<YAML::BeginMap;
  for(std::size_t i=0;i<result.cameras.size();++i) {
    const auto& camera=result.cameras[i]; const auto& c=camera.optical;
    out<<YAML::Key<<c.id<<YAML::Value<<YAML::BeginMap
       <<YAML::Key<<"camera_model"<<YAML::Value<<c.model
       <<YAML::Key<<"distortion_model"<<YAML::Value<<c.distortion
       <<YAML::Key<<"intrinsics"<<YAML::Value<<YAML::Flow<<c.intrinsics
       <<YAML::Key<<"distortion_coeffs"<<YAML::Value<<YAML::Flow<<c.coefficients
       <<YAML::Key<<"resolution"<<YAML::Value<<YAML::Flow<<std::vector<std::uint32_t>{c.width,c.height};
    if(i) {
      const auto transform=relative(camera.T_cam_reference,result.cameras[i-1].T_cam_reference);
      out<<YAML::Key<<"T_cn_cnm1"<<YAML::Value<<YAML::BeginSeq;
      for(const auto& row:transform) out<<YAML::Flow<<std::vector<double>(row.begin(),row.end());
      out<<YAML::EndSeq;
    }
    out<<YAML::EndMap;
  }
  out<<YAML::EndMap;
  if(!out.good()) throw Error(5,"Cannot serialize rig calibration");
  publish(path,"# GS camera-camera estimate. T_cn_cnm1 maps preceding selected camera into this camera.\n# See calibration.json/report.pdf for validation limits. No IMU or timing calibration.\n"+std::string(out.c_str())+'\n');
}
void write_rig_pdf(const fs::path& path,const RigCalibration& result) {
  std::vector<std::string> pages;
  std::ostringstream overview;
  text(overview,45,795,"Camera-camera calibration",20);
  text(overview,45,765,"Reference: "+result.reference_camera+"   |   Global shutter / exact synchronized timestamps");
  text(overview,45,735,"Optimizer converged. Calibration quality remains provisional.",12);
  text(overview,45,710,"No full observability test or marginalized covariance is available.");
  text(overview,45,690,"No IMU extrinsics, clock offset or rolling-shutter readout was estimated.");
  text(overview,45,655,"Training views: "+result.diagnostics.at("training_views").dump()+
    "   Held-out views: "+result.diagnostics.at("held_out_views").dump());
  text(overview,45,635,"Held-out target pose is fitted to one camera; other views test the rig.");
  text(overview,45,615,"These views belong to the same recording, not an independent acquisition.");
  double y=565;
  text(overview,45,y,"Camera       Training RMS [px]       Held-out RMS [px]",12); y-=28;
  for(const auto& camera:result.cameras) {
    text(overview,45,y,camera.optical.id); text(overview,160,y,number(camera.training_rms_px));
    text(overview,330,y,number(camera.held_out_rms_px)); y-=22;
    if(y<100) { pages.push_back(overview.str()); overview.str(""); overview.clear(); y=790; }
  }
  text(overview,45,y-25,"Huber loss scale: 2 px. Large residuals are downweighted, not hidden.");
  text(overview,45,y-45,"RMS is sqrt(sum of squared 2D corner errors / number of corners).");
  pages.push_back(overview.str());
  for(std::size_t i=0;i<result.cameras.size();++i) {
    const auto& c=result.cameras[i]; std::ostringstream page;
    text(page,45,795,c.optical.id+" - "+c.optical.distortion,18);
    text(page,45,763,"Resolution: "+std::to_string(c.optical.width)+" x "+std::to_string(c.optical.height));
    text(page,45,738,"fx / fy: "+number(c.optical.intrinsics[0])+" / "+number(c.optical.intrinsics[1]));
    text(page,45,718,"cx / cy: "+number(c.optical.intrinsics[2])+" / "+number(c.optical.intrinsics[3]));
    std::string distortion="Distortion:"; for(double v:c.optical.coefficients) distortion+=' '+number(v);
    text(page,45,698,distortion);
    text(page,45,665,"T_cam_reference: reference coordinates -> camera coordinates",12);
    for(unsigned r=0;r<4;++r) {
      std::string line; for(double v:c.T_cam_reference[r]) line+=number(v)+"    ";
      text(page,60,641-20*r,line);
    }
    text(page,45,550,"Camera optical center in reference frame [m]:");
    text(page,60,530,number(c.center_in_reference_m[0])+"   "+number(c.center_in_reference_m[1])+"   "+number(c.center_in_reference_m[2]));
    text(page,45,490,"Training residual histogram (all used corners)",12);
    const auto bins=result.diagnostics.at("metrics").at(i).at("training_residual_histogram").get<std::array<std::size_t,6>>();
    const auto maximum=*std::max_element(bins.begin(),bins.end());
    const std::array<std::string,6> labels{"0 - 0.25","0.25 - 0.5","0.5 - 1","1 - 2","2 - 5",">= 5"};
    for(unsigned b=0;b<6;++b) {
      const double height=maximum?160.*static_cast<double>(bins[b])/static_cast<double>(maximum):0.;
      page<<"0.18 0.42 0.68 rg "<<55+80*b<<" 280 45 "<<height<<" re f\n";
      text(page,55+80*b,263,labels[b],9); text(page,55+80*b,450,std::to_string(bins[b]),9);
    }
    text(page,45,220,"Pixels; numerical inversion checked only over the observed angular field.");
    text(page,45,195,"Detailed poses, downweighted corners and failed seeds: calibration.json");
    pages.push_back(page.str());
  }
  std::vector<std::string> objects(3);
  objects[0]="<< /Type /Catalog /Pages 2 0 R >>";
  objects[2]="<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>";
  std::string kids;
  for(const auto& page:pages) {
    const auto id=objects.size()+1; kids+=std::to_string(id)+" 0 R ";
    objects.push_back("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 595 842] /Resources << /Font << /F1 3 0 R >> >> /Contents "+std::to_string(id+1)+" 0 R >>");
    objects.push_back("<< /Length "+std::to_string(page.size())+" >>\nstream\n"+page+"endstream");
  }
  objects[1]="<< /Type /Pages /Count "+std::to_string(pages.size())+" /Kids ["+kids+"] >>";
  std::ostringstream pdf; pdf<<"%PDF-1.4\n"; std::vector<std::streamoff> offsets{0};
  for(std::size_t i=0;i<objects.size();++i) { offsets.push_back(pdf.tellp()); pdf<<i+1<<" 0 obj\n"<<objects[i]<<"\nendobj\n"; }
  const auto xref=pdf.tellp(); pdf<<"xref\n0 "<<objects.size()+1<<"\n0000000000 65535 f \n";
  for(std::size_t i=1;i<offsets.size();++i) pdf<<std::setw(10)<<std::setfill('0')<<offsets[i]<<" 00000 n \n";
  pdf<<"trailer\n<< /Size "<<objects.size()+1<<" /Root 1 0 R >>\nstartxref\n"<<xref<<"\n%%EOF\n";
  publish(path,pdf.str());
}
} // namespace calib
