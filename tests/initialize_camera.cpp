#include "calib/initialization.hpp"
#include <iostream>

int main(int argc,char** argv) {
  try {
    calib::Json input; std::cin>>input;
    const auto& t=input.at("target");
    const calib::AprilGridGeometry geometry({"tag36h11",t.at("rows"),t.at("cols"),0,t.at("size"),t.at("spacing"),2});
    std::vector<calib::FrameObservations> frames;
    for(const auto& v:input.at("frames")) {
      calib::FrameObservations frame;
      frame.camera="cam0"; frame.timestamp_ns=v.at("timestamp_ns");
      frame.resolution=input.at("resolution").get<std::array<std::uint32_t,2>>();
      for(const auto& p:v.at("points")) frame.corners.push_back({p.at("point_id"),p.at("pixel").get<std::array<double,2>>()});
      frames.push_back(std::move(frame));
    }
    const auto name=input.at("model").get<std::string>();
    if(name!="equidistant" && name!="radtan") throw std::runtime_error("Unknown model");
    const auto result=calib::initialize_camera(geometry,frames,name=="radtan"?calib::DistortionModel::radtan:calib::DistortionModel::equidistant,
      [&]{return input.value("cancelled",false);});
    if(argc==2) {
      calib::write_initial_camchain(argv[1],std::span(&result,1));
      const auto imported=calib::read_camchain(argv[1]);
      if(imported.size()!=1 || imported[0].intrinsics!=result.camera.intrinsics || imported[0].coefficients!=result.camera.coefficients)
        throw std::runtime_error("Initial camchain roundtrip failed");
    }
    std::cout<<calib::initialization_json(result).dump()<<'\n'; return 0;
  } catch(const calib::Error& error) { std::cerr<<error.what()<<'\n'; return error.code; }
    catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 5; }
}
