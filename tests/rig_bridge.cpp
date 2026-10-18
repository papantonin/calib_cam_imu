#include "calib/rig.hpp"
#include <iostream>

int main(int argc,char** argv) {
  try {
    calib::Json input; std::cin>>input;
    const calib::AprilGridGeometry geometry({"tag36h11",6,6,0,.04,.3,2});
    std::vector<calib::InitialCalibration> initial;
    for(const auto& c:input.at("cameras")) {
      calib::InitialCalibration value;
      value.camera={c.at("id"),"pinhole",c.at("model"),c.at("intrinsics").get<std::vector<double>>(),
        c.at("distortion").get<std::vector<double>>(),c.at("resolution")[0],c.at("resolution")[1],0,false};
      initial.push_back(value);
    }
    std::vector<calib::SynchronizedView> views;
    for(const auto& v:input.at("views")) {
      calib::SynchronizedView view{v.at("timestamp_ns"),v.at("held_out"),{}};
      for(const auto& f:v.at("cameras")) {
        calib::FrameObservations frame;
        frame.camera=f.at("camera"); frame.timestamp_ns=f.value("timestamp_ns",view.timestamp_ns);
        frame.resolution=f.at("resolution").get<std::array<std::uint32_t,2>>();
        for(const auto& p:f.at("points")) frame.corners.push_back({p.at("point_id"),p.at("pixel").get<std::array<double,2>>()});
        view.cameras.push_back(frame);
      }
      views.push_back(view);
    }
    const auto result=calib::calibrate_rig(geometry,initial,views,{input.value("threads",1u),[&]{return input.value("cancelled",false);}});
    if(argc==2) {
      const calib::fs::path root=argv[1]; calib::fs::create_directory(root);
      calib::write_rig_camchain(root/"camchain.yaml",result);
      calib::write_rig_pdf(root/"report.pdf",result);
      const auto imported=calib::read_camchain(root/"camchain.yaml");
      if(imported.size()!=result.cameras.size()) throw std::runtime_error("Camchain optical import failed");
    }
    std::cout<<calib::rig_json(result).dump()<<'\n'; return 0;
  } catch(const calib::Error& error) { std::cerr<<error.what()<<'\n'; return error.code; }
    catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 5; }
}
