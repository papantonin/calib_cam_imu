#include "calib/core.hpp"
#include "calib/observations.hpp"
#include "calib/initialization.hpp"
#include "calib/rig.hpp"
#include <CLI/CLI.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <future>
#include <iostream>
#include <map>
#include <optional>
#include <thread>
#include <sys/resource.h>

namespace {
static_assert(std::atomic_bool::is_always_lock_free);
std::atomic_bool interrupted{false};
void on_signal(int) { interrupted.store(true,std::memory_order_relaxed); }
void write_json(const calib::fs::path& path, const calib::Json& value) {
  const auto temporary = calib::fs::path(path.string()+".incomplete");
  std::ofstream stream(temporary);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream << value.dump(2) << '\n';
  stream.close();
  calib::fs::rename(temporary,path);
}
}
int main(int argc, char** argv) {
  using namespace calib;
#ifdef CALIB_REQUIRE_IMU
  constexpr bool require_imu=true;
#else
  constexpr bool require_imu=false;
#endif
  CLI::App app("Camera calibration: automatic synchronized GS camera-camera solve, inspection and initialization. IMU/RS solve not yet implemented.");
  app.set_version_flag("--version",std::string(CALIB_VERSION)+"; AprilTag "+CALIB_APRILTAG_VERSION+" ("+CALIB_APRILTAG_COMMIT+")");
  std::string dataset_path, target_path, cam_path, output_path, tag_family;
  std::vector<std::string> cameras,models;
  bool check=false, detect=false, initialize=false, solve_rig=false;
  unsigned init_max_views=60;
  unsigned threads=0, memory_mib=1024;
  double decimate=1.0;
  int tag_border_bits=1;
  app.add_option("--dataset",dataset_path,"Native camN/<timestamp_ns>.png dataset")->required();
  app.add_option("--cameras",cameras,"Selected cameras, in reference order")->expected(-1);
  auto check_option=app.add_flag("--check-dataset",check,"Validate timestamps, PNG payloads and (for VI) IMU CSV");
  auto detect_option=app.add_flag("--detect-only",detect,"Detect target tags and export observations; no calibration");
  check_option->excludes(detect_option);
  auto init_option=app.add_flag("--initialize-only",initialize,"Detect and estimate initial GS intrinsics/target poses; no rig or IMU calibration");
  init_option->excludes(check_option); init_option->excludes(detect_option);
  auto models_option=app.add_option("--models",models,"Optical models: pinhole-radtan or pinhole-equi; one for all, or one per camera")->expected(-1);
  auto views_option=app.add_option("--init-max-views",init_max_views,"Maximum initial views per camera and synchronized rig groups; 20% held out")->check(CLI::Range(10,200));
  auto target_option=app.add_option("--target",target_path,"Explicit AprilGrid geometry, family and border YAML");
  app.add_option("--tag-family",tag_family,"Diagnostic detection without target geometry: tag36h11 or tagStandard41h12")->excludes(target_option);
  auto border_option=app.add_option("--tag-border-bits",tag_border_bits,"Diagnostic tag border: 1 native, 2 legacy tag36h11")->check(CLI::Range(1,2));
  app.add_option("--cam",cam_path,"Inspect camchain intrinsics, resolution and signed line_delay");
  app.add_option("--output",output_path,"New output directory (required for detection/initialization/calibration; never overwritten)");
  app.add_option("--threads",threads,"Global CPU budget for images, initialization and solver; 0 = hardware concurrency")->check(CLI::Range(0,256));
  app.add_option("--memory-budget-mb",memory_mib,"Estimated image workspace budget, not a hard RSS cap")->check(CLI::Range(32,65536));
  auto decimate_option=app.add_option("--quad-decimate",decimate,"Detection decimation [1,4]; corners remain in original pixels")->check(CLI::Range(1.0,4.0));
  try { app.parse(argc,argv); }
  catch(const CLI::ParseError& error) { const int code=app.exit(error); return code==0?0:2; }
  bool output_created=false;
  try {
    if(!check && !detect && !initialize) {
      if(require_imu) throw Error(8,"IMU calibration solver is not implemented yet. Use --check-dataset, --detect-only or --initialize-only.");
      solve_rig=true; initialize=true;
    }
    if(initialize && (target_path.empty() || output_path.empty() || models.empty()))
      throw Error(2,"Camera calibration/initialization requires --target, --models and --output");
    if(!initialize && (models_option->count() || views_option->count())) throw Error(2,"--models and --init-max-views require camera calibration or --initialize-only");
    if(initialize && !cam_path.empty()) throw Error(8,"Initialization from --cam, including rolling shutter input, is not supported yet");
    if(initialize) detect=true;
    if(detect && ((target_path.empty() && tag_family.empty()) || output_path.empty())) throw Error(2,"--detect-only requires --output and either --target or diagnostic --tag-family");
    if(border_option->count() && (!detect || tag_family.empty())) throw Error(2,"--tag-border-bits requires diagnostic --tag-family");
    if(check && (!target_path.empty() || !tag_family.empty() || decimate_option->count())) throw Error(2,"--target, --tag-family and --quad-decimate require --detect-only");
    std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal);
    const auto start=std::chrono::steady_clock::now();
    NativeDirectoryReader reader(dataset_path);
    const auto dataset=reader.index(cameras,require_imu);
    std::optional<RigViewSampler> rig_sampler;
    if(solve_rig) {
      if(dataset.cameras.size()<2) throw Error(3,"Camera-camera calibration needs at least two selected cameras");
      rig_sampler.emplace(dataset,init_max_views);
    }
    std::map<std::string,InitialViewSampler> samplers;
    std::vector<DistortionModel> model_kinds;
    if(initialize) {
      if(models.size()!=1 && models.size()!=dataset.cameras.size()) throw Error(2,"Provide one --models value for all cameras, or exactly one per selected camera");
      for(std::size_t i=0;i<dataset.cameras.size();++i) {
        const auto& name=models[models.size()==1?0:i];
        if(name!="pinhole-radtan" && name!="pinhole-equi") throw Error(2,"Unsupported initialization model: "+name);
        model_kinds.push_back(name=="pinhole-radtan"?DistortionModel::radtan:DistortionModel::equidistant);
        samplers.emplace(dataset.cameras[i].id,InitialViewSampler(dataset.cameras[i].frames.size(),init_max_views));
      }
    }
    Json summary=dataset_summary(dataset);
    summary["schema_version"]=1;
    summary["mode"]=solve_rig?"camera_camera_calibration":initialize?"initialization":detect?"detection":"dataset_check";
    summary["software"]={{"version",CALIB_VERSION},{"apriltag_version",CALIB_APRILTAG_VERSION},{"apriltag_commit",CALIB_APRILTAG_COMMIT}};
    summary["calibration"]={{"status","not_performed"},{"quality","not_assessed"}};
    summary["warnings"]=Json::array({"Structural checks and tag counts do not establish calibration observability or accuracy."});
    if(require_imu) summary["warnings"].push_back("IMU overlap uses raw timestamps; row timing, offsets, sensor noise and motion excitation are not assessed yet.");
    if(!cam_path.empty()) {
      const auto calibration=read_camchain(cam_path);
      summary["camchain"]=Json::array();
      for(const auto& stream:dataset.cameras) {
        const auto camera=std::ranges::find(calibration,stream.id,&CameraCalibration::id);
        if(camera==calibration.end()) throw Error(2,"Selected camera missing from camchain: "+stream.id);
        const auto shape=stream.frames.front().image;
        if(shape.width!=camera->width || shape.height!=camera->height) throw Error(2,"Camchain resolution differs from dataset: "+stream.id);
        summary["camchain"].push_back({{"camera",camera->id},{"camera_model",camera->model},{"distortion_model",camera->distortion},
          {"line_delay_present",camera->has_line_delay},{"line_delay_s",camera->line_delay_s},
          {"readout_time_s",(camera->height-1)*std::abs(camera->line_delay_s)},
          {"shutter",camera->line_delay_s==0?"global":"rolling"},{"time_convention","t_imu = t_image + timeshift_cam_imu + y * line_delay"}});
      }
      summary["warnings"].push_back("Camchain inspection validates intrinsics and row timing only; extrinsics and offsets are not interpreted yet.");
    }
    std::optional<Target> target;
    if(detect && !target_path.empty()) target=read_target(target_path);
    std::optional<AprilGridGeometry> geometry;
    std::vector<const FrameRef*> frames;
    std::size_t max_pixels=0;
    for(const auto& camera:dataset.cameras) for(const auto& frame:camera.frames) {
      frames.push_back(&frame);
      max_pixels=std::max(max_pixels,std::size_t(frame.image.width)*frame.image.height);
    }
    const std::size_t budget=std::size_t(memory_mib)*1024*1024;
    // Conservative scheduling estimate. AprilTag's data-dependent allocations are not a hard cap.
    const std::size_t worker_bytes=max_pixels*(detect?64:8)+16*1024*1024;
    if(worker_bytes>budget) throw Error(2,"Largest image exceeds estimated workspace budget; increase --memory-budget-mb");
    const auto requested=threads?threads:std::max(1u,std::thread::hardware_concurrency());
    const auto workers=std::min({std::size_t(requested),budget/worker_bytes,frames.size()});
    summary["execution"]={{"requested_threads",requested},{"workers",workers},{"memory_budget_mib",memory_mib},
      {"estimated_worker_bytes",worker_bytes},{"memory_budget_kind","image_workspace_estimate_not_hard_rss_limit"}};
    std::vector<std::unique_ptr<Detector>> detectors;
    if(detect) {
      for(std::size_t i=0;i<workers;++i) detectors.push_back(target?std::make_unique<Detector>(*target,decimate):std::make_unique<Detector>(tag_family,decimate,tag_border_bits));
      if(target) geometry.emplace(*target);
      if(target) summary["target"]={{"family",target->family},{"rows",target->rows},{"cols",target->cols},{"first_id",target->first_id},
        {"tag_size_m",target->size_m},{"spacing_ratio",target->spacing_ratio},{"border_bits",target->border_bits}};
      else {
        summary["target"]={{"family",tag_family},{"geometry",nullptr},{"status","unknown_diagnostic_only"},{"border_bits",tag_border_bits}};
        summary["warnings"].push_back("Diagnostic family scan: target dimensions, layout, physical scale and border compatibility are not validated.");
      }
      summary["detection"]={{"quad_decimate",decimate},{"max_hamming",1},{"refine_edges",true},{"mono16_to_gray8","floor(raw / 256), no gamma or per-frame normalization"},
        {"corner_order","AprilTag canonical p[0..3]: (-1,+1),(+1,+1),(+1,-1),(-1,-1)"}};
    }
    std::ofstream observations;
    std::ofstream correspondences;
    if(!output_path.empty()) {
      if(!fs::create_directory(output_path)) throw Error(5,"Output directory already exists: "+output_path);
      output_created=true;
      if(detect) {
        observations.open(fs::path(output_path)/"detections.jsonl.incomplete");
        observations.exceptions(std::ios::failbit | std::ios::badbit);
        if(geometry) {
          write_json(fs::path(output_path)/"target_geometry.json",geometry_json(*geometry));
          correspondences.open(fs::path(output_path)/"observations.jsonl.incomplete");
          correspondences.exceptions(std::ios::failbit | std::ios::badbit);
        }
      }
    }
    std::size_t tag_count=0, detected_frames=0, rejected_count=0;
    std::size_t accepted_corners=0, rejected_observation_tags=0;
    std::map<std::string,Json> camera_counts;
    for(const auto& camera:dataset.cameras) camera_counts[camera.id]={
      {"frames",camera.frames.size()},{"frames_with_target",0},{"tags",0},{"rejected_outside_grid",0}};
    auto last_progress=start;
    for(std::size_t begin=0;begin<frames.size();begin+=workers) {
      if(interrupted) throw Error(130,"Interrupted; incomplete output must not be reused");
      std::vector<std::future<Json>> tasks;
      const auto end=std::min(begin+workers,frames.size());
      for(std::size_t i=begin;i<end;++i) tasks.push_back(std::async(std::launch::async,[&,i,begin] {
        try {
          auto image=reader.decode(*frames[i],max_pixels);
          if(!detect) return Json{};
          auto tags=detectors[i-begin]->detect(image);
          Json correspondence_record;
          if(geometry) correspondence_record=observations_json(make_observations(*geometry,*frames[i],tags));
          Json rejected=Json::array();
          std::erase_if(tags,[&](const Tag& tag){
            const bool outside=target && (tag.id<target->first_id || tag.id>=target->first_id+target->rows*target->cols);
            if(outside) rejected.push_back({{"id",tag.id},{"reason","outside_grid_id_range"},
              {"hamming",tag.hamming},{"decision_margin",tag.margin},{"corners_px",tag.corners}});
            return outside;
          });
          auto result=tags_json(*frames[i],tags);
          result["rejected_outside_grid"]=rejected.size();
          result["rejected_tags"]=std::move(rejected);
          result["status"]=tags.empty()?"no_target_tags":"detected";
          if(geometry) result["_correspondences"]=std::move(correspondence_record);
          return result;
        } catch(const Error& error) { throw Error(error.code,frames[i]->path.string()+": "+error.what()); }
      }));
      // Ordered consumption gives reproducible output independent of worker completion order.
      for(auto& task:tasks) {
        auto result=task.get();
        if(detect) {
          if(geometry) {
            const auto& record=result.at("_correspondences");
            accepted_corners+=record["points"].size();
            rejected_observation_tags+=record["rejected_tags"].size();
            correspondences << record.dump() << '\n';
            if(initialize) {
              FrameObservations frame;
              frame.camera=record.at("camera"); frame.timestamp_ns=record.at("timestamp_ns");
              frame.resolution=record.at("resolution").get<std::array<std::uint32_t,2>>();
              for(const auto& point:record.at("points")) frame.corners.push_back({point.at("point_id"),point.at("pixel").get<std::array<double,2>>()});
              if(rig_sampler) {
                rig_sampler->consider(frame);
                // A rig validation timestamp must not influence intrinsic initialization.
                if(rig_sampler->held_out(frame.timestamp_ns)) frame.corners.clear();
              }
              samplers.at(frame.camera).consider(std::move(frame));
            }
            result.erase("_correspondences");
          }
          tag_count+=result["tags"].size();
          detected_frames+=!result["tags"].empty();
          rejected_count+=result["rejected_outside_grid"].get<std::size_t>();
          auto& counts=camera_counts.at(result["camera"].get<std::string>());
          counts["frames_with_target"]=counts["frames_with_target"].get<std::size_t>()+!result["tags"].empty();
          counts["tags"]=counts["tags"].get<std::size_t>()+result["tags"].size();
          counts["rejected_outside_grid"]=counts["rejected_outside_grid"].get<std::size_t>()+result["rejected_outside_grid"].get<std::size_t>();
          observations << result.dump() << '\n';
        }
      }
      if(std::chrono::steady_clock::now()-last_progress>std::chrono::seconds(2)) {
        std::cerr << "Processed " << end << '/' << frames.size() << " frames\n";
        last_progress=std::chrono::steady_clock::now();
      }
    }
    if(interrupted) throw Error(130,"Interrupted; incomplete output must not be reused");
    bool missing_camera_tags=false;
    if(detect) {
      observations.close();
      if(geometry) {
        correspondences.close();
        fs::rename(fs::path(output_path)/"observations.jsonl.incomplete",fs::path(output_path)/"observations.jsonl");
        summary["observations"]={{"schema_version",1},{"accepted_corners",accepted_corners},
          {"rejected_tags",rejected_observation_tags},{"geometry_file","target_geometry.json"},
          {"observations_file","observations.jsonl"},{"calibration_suitability","not_assessed"}};
      }
      fs::rename(fs::path(output_path)/"detections.jsonl.incomplete",fs::path(output_path)/"detections.jsonl");
      summary["detection"]["tags"]=tag_count;
      summary["detection"]["frames_with_target"]=detected_frames;
      summary["detection"]["rejected_outside_grid"]=rejected_count;
      summary["detection"]["cameras"]=camera_counts;
      for(const auto& [id,counts]:camera_counts) if(counts["tags"].get<std::size_t>()==0) {
        missing_camera_tags=true;
        summary["warnings"].push_back("No target tags in "+id+"; calibration involving this camera is impossible with these observations.");
      }
    }
    if(initialize) {
      std::vector<InitialCalibration> initial;
      for(std::size_t begin=0;begin<dataset.cameras.size();begin+=workers) {
        std::vector<std::future<InitialCalibration>> tasks;
        for(std::size_t i=begin;i<std::min(begin+workers,dataset.cameras.size());++i) {
          std::cerr<<"Initializing intrinsics for "<<dataset.cameras[i].id<<'\n';
          tasks.push_back(std::async(std::launch::async,[&,i] {
            try { return initialize_camera(*geometry,samplers.at(dataset.cameras[i].id).selected(),model_kinds[i],[]{return interrupted!=0;}); }
            catch(const Error& error) { throw Error(error.code,dataset.cameras[i].id+": "+error.what()); }
          }));
        }
        for(auto& task:tasks) initial.push_back(task.get());
      }
      if(interrupted) throw Error(130,"Initialization interrupted before publication");
      Json report={{"schema_version",1},{"status","initial_estimates"},{"cameras",Json::array()},
        {"selection",{{"method","maximum_corners_per_temporal_frame_bin"},{"max_views_per_camera",init_max_views},
          {"minimum_corners_per_view",12},{"held_out_rule","every fifth selected view"}}}};
      for(const auto& camera:initial) report["cameras"].push_back(initialization_json(camera));
      write_json(fs::path(output_path)/"initialization.json",report);
      write_initial_camchain(fs::path(output_path)/"initial-camchain.yaml",initial);
      summary["initialization"]={{"status","initial_estimates"},{"report","initialization.json"},{"camchain","initial-camchain.yaml"}};
      if(solve_rig) {
        std::cerr<<"Optimizing camera-camera extrinsics and intrinsics\n";
        const auto rig=calibrate_rig(*geometry,initial,rig_sampler->selected(),
          {static_cast<unsigned>(workers),[]{return interrupted.load(std::memory_order_relaxed);}});
        if(interrupted) throw Error(130,"Interrupted before rig publication");
        write_json(fs::path(output_path)/"calibration.json",rig_json(rig));
        write_rig_camchain(fs::path(output_path)/"camchain.yaml",rig);
        write_rig_pdf(fs::path(output_path)/"report.pdf",rig);
        summary["calibration"]={{"status","converged"},{"quality","provisional_not_certified"},
          {"reference_camera",rig.reference_camera},{"result","calibration.json"},{"camchain","camchain.yaml"},{"report","report.pdf"}};
        summary["warnings"].push_back("Synchronized global-shutter rig estimate. Full observability, covariance, IMU extrinsics and temporal calibration remain unavailable.");
      } else summary["warnings"].push_back("Initial GS optical estimates only: no rig/IMU extrinsics, timing, covariance or certified calibration quality.");
    }
    summary["status"]=missing_camera_tags?"insufficient_data":"completed";
    summary["decoded_frames"]=frames.size();
    summary["execution"]["elapsed_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    rusage usage{};
    if(getrusage(RUSAGE_SELF,&usage)==0) summary["execution"]["peak_rss_kib"]=usage.ru_maxrss;
    if(output_created) write_json(fs::path(output_path)/"summary.json",summary);
    std::cout << summary.dump(2) << '\n';
    return missing_camera_tags?3:0;
  } catch(const std::exception& error) {
    const auto* known=dynamic_cast<const Error*>(&error);
    const int code=known?known->code:5;
    std::cerr << "error: " << error.what() << '\n';
    if(output_created) {
      try { write_json(fs::path(output_path)/"error.json",{{"status","failed"},{"exit_code",code},{"message",error.what()}}); }
      catch(...) { /* Preserve the original error if the filesystem is also unavailable. */ }
    }
    return code;
  }
}
