#include "calib/vi.hpp"
#include <iostream>

int main(int argc,char** argv) {
  try {
    calib::Json input; std::cin>>input;
    const calib::AprilGridGeometry geometry({"tag36h11",6,6,0,input.value("tag_size_m",.04),.3,2});
    const auto& source=input.at("camera");
    const calib::CameraCalibration camera{source.value("id",std::string("cam0")),"pinhole",source.at("model"),
      source.at("intrinsics").get<std::vector<double>>(),source.at("distortion").get<std::vector<double>>(),
      source.at("resolution")[0],source.at("resolution")[1],source.value("line_delay",0.),source.contains("line_delay")};
    std::vector<calib::FrameObservations> frames;
    for(const auto& frame:input.at("frames")) {
      calib::FrameObservations observation;
      observation.camera=frame.value("camera",camera.id); observation.timestamp_ns=frame.at("timestamp_ns");
      observation.resolution=frame.value("resolution",std::array<std::uint32_t,2>{camera.width,camera.height});
      for(const auto& point:frame.at("points"))
        observation.corners.push_back({point.at("point_id"),point.at("pixel").get<std::array<double,2>>()});
      frames.push_back(std::move(observation));
    }
    std::vector<calib::ImuSample> imu;
    for(const auto& sample:input.at("imu")) imu.push_back({sample.at("timestamp_ns"),sample.at("values").get<std::array<double,6>>()});
    calib::ViOptions options;
    options.noise={input.at("noise").at("gyroscope_noise_density"),input.at("noise").at("accelerometer_noise_density"),
      input.at("noise").value("gyroscope_random_walk",0.0),input.at("noise").value("accelerometer_random_walk",0.0)};
    options.knot_interval_s=input.value("knot_interval_s",.1); options.max_offset_s=input.value("max_offset_s",.05);
    options.bias_knot_interval_s=input.value("bias_knot_interval_s",.1);
    options.pixel_sigma=input.value("pixel_sigma",.8);
    options.max_iterations=input.value("max_iterations",80u); options.threads=input.value("threads",1u);
    options.bootstrap_replicates=input.value("bootstrap_replicates",0u);
    options.bootstrap_block_s=input.value("bootstrap_block_s",0.0);
    options.bootstrap_seed=input.value("bootstrap_seed",std::uint64_t{20261020});
    options.bootstrap_max_iterations=input.value("bootstrap_max_iterations",40u);
    options.cancelled=[&]{return input.value("cancelled",false);};
    const auto result=calib::calibrate_vi(geometry,camera,frames,imu,options);
    if(argc==2) {
      const calib::fs::path directory=argv[1]; calib::fs::create_directory(directory);
      calib::write_vi_camchain(directory/"camchain.yaml",result); calib::write_vi_pdf(directory/"report.pdf",result);
      const auto imported=calib::read_camchain(directory/"camchain.yaml");
      if(imported.size()!=1||imported[0].intrinsics!=camera.intrinsics) throw std::runtime_error("Camchain optical import failed");
    }
    auto output=calib::vi_json(result);
    output["bootstrap"]=result.bootstrap;
    output["bridge_truth_access"]={{"T_cam_imu",result.T_cam_imu},{"offset",result.timeshift_cam_imu},
      {"gyro_bias",result.gyro_bias},{"accel_bias",result.accel_bias},{"gravity_world",result.gravity_world}};
    std::cout<<output.dump()<<'\n'; return 0;
  } catch(const calib::Error& error) {std::cerr<<error.what()<<'\n';return error.code;}
    catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 5;}
}
