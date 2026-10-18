#include "calib/rig.hpp"
#include "calib/rig_residual.hpp"
#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>

namespace calib {
RigViewSampler::RigViewSampler(const Dataset& dataset,std::size_t limit) {
  if(limit<10 || limit>200) throw Error(2,"Rig view budget must be in [10,200]");
  std::map<std::int64_t,std::size_t> counts;
  for(const auto& camera:dataset.cameras) for(const auto& frame:camera.frames) ++counts[frame.timestamp_ns];
  std::vector<std::int64_t> common;
  for(const auto& [stamp,count]:counts) if(count>=2) common.push_back(stamp);
  if(common.size()<10) throw Error(3,"Need at least ten exact common camera timestamps; asynchronous rig calibration is not implemented");
  const auto size=std::min(limit,common.size());
  for(std::size_t i=0;i<size;++i) {
    const auto stamp=common[i*(common.size()-1)/(size-1)];
    views_.emplace(stamp,SynchronizedView{stamp,i%5==4,{}});
  }
}
void RigViewSampler::consider(const FrameObservations& frame) {
  const auto found=views_.find(frame.timestamp_ns);
  if(found!=views_.end() && frame.corners.size()>=12) found->second.cameras.push_back(frame);
}
bool RigViewSampler::held_out(std::int64_t stamp) const {
  const auto found=views_.find(stamp); return found!=views_.end() && found->second.held_out;
}
std::vector<SynchronizedView> RigViewSampler::selected() const {
  std::vector<SynchronizedView> views;
  for(const auto& [stamp,view]:views_) { (void)stamp; views.push_back(view); }
  return views;
}
namespace {
using Matrix=Eigen::Matrix4d;
using Pose=std::array<double,6>;
Matrix matrix(const TransformMatrix& source) {
  Matrix result;
  for(unsigned r=0;r<4;++r) for(unsigned c=0;c<4;++c) result(r,c)=source[r][c];
  return result;
}
TransformMatrix array(const Matrix& source) {
  TransformMatrix result;
  for(unsigned r=0;r<4;++r) for(unsigned c=0;c<4;++c) result[r][c]=source(r,c);
  return result;
}
Pose pack(const Matrix& transform) {
  const Eigen::AngleAxisd rotation(transform.block<3,3>(0,0));
  const auto vector=rotation.axis()*rotation.angle();
  return {vector.x(),vector.y(),vector.z(),transform(0,3),transform(1,3),transform(2,3)};
}
Matrix unpack(const Pose& pose) {
  const Eigen::Vector3d vector(pose[0],pose[1],pose[2]);
  Matrix result=Matrix::Identity();
  const double angle=vector.norm();
  if(angle>0) result.block<3,3>(0,0)=Eigen::AngleAxisd(angle,vector/angle).toRotationMatrix();
  result.block<3,1>(0,3)=Eigen::Vector3d(pose[3],pose[4],pose[5]); return result;
}
Matrix inverse(const Matrix& transform) {
  Matrix result=Matrix::Identity();
  result.block<3,3>(0,0)=transform.block<3,3>(0,0).transpose();
  result.block<3,1>(0,3)=-result.block<3,3>(0,0)*transform.block<3,1>(0,3); return result;
}
Matrix medoid(const std::vector<Matrix>& candidates) {
  double best=std::numeric_limits<double>::infinity(); std::size_t selected=0;
  for(std::size_t i=0;i<candidates.size();++i) {
    double cost=0;
    for(const auto& other:candidates) {
      const Matrix delta=candidates[i]*inverse(other);
      cost+=Eigen::AngleAxisd(delta.block<3,3>(0,0)).angle()/.05+delta.block<3,1>(0,3).norm()/.05;
    }
    if(cost<best) { best=cost; selected=i; }
  }
  return candidates[selected];
}
void check_cancelled(const RigOptions& options) {
  if(options.cancelled && options.cancelled()) throw Error(130,"Rig calibration interrupted");
}
struct CancelCallback final:ceres::IterationCallback {
  const RigOptions& options;
  explicit CancelCallback(const RigOptions& value):options(value) {}
  ceres::CallbackReturnType operator()(const ceres::IterationSummary&) override {
    return options.cancelled && options.cancelled()?ceres::SOLVER_ABORT:ceres::SOLVER_CONTINUE;
  }
};
struct WorkingView {
  const SynchronizedView* source{};
  std::map<std::size_t,Matrix> seeds;
  Pose target{};
};
}

RigCalibration calibrate_rig(const AprilGridGeometry& geometry,std::span<const InitialCalibration> initial,
                             std::span<const SynchronizedView> views,const RigOptions& options) {
  if(initial.size()<2) throw Error(3,"Camera-camera calibration requires at least two cameras");
  if(views.size()<10 || views.size()>200 || options.threads<1 || options.threads>256)
    throw Error(2,"Invalid rig view or CPU budget");
  check_cancelled(options);
  const auto count=initial.size();
  std::map<std::string,std::size_t> indices;
  std::vector<CameraModel::Parameters> optics;
  std::vector<DistortionModel> kinds;
  for(std::size_t i=0;i<count;++i) {
    if(!indices.emplace(initial[i].camera.id,i).second) throw Error(2,"Duplicate rig camera");
    if(initial[i].camera.line_delay_s!=0) throw Error(8,"Rolling shutter rig calibration is not implemented");
    const auto optical=make_camera_model(initial[i].camera);
    optics.push_back(optical.parameters()); kinds.push_back(optical.distortion());
  }
  Json rejected=Json::array(),edges=Json::array();
  std::vector<WorkingView> working;
  std::map<std::pair<std::size_t,std::size_t>,std::vector<Matrix>> candidates;
  std::int64_t previous=-1;
  for(const auto& view:views) {
    check_cancelled(options);
    if(view.timestamp_ns<=previous) throw Error(2,"Rig timestamps must increase strictly");
    previous=view.timestamp_ns;
    WorkingView item; item.source=&view;
    std::set<std::string> seen;
    for(const auto& frame:view.cameras) {
      if(frame.timestamp_ns!=view.timestamp_ns || !indices.contains(frame.camera) || !seen.insert(frame.camera).second)
        throw Error(2,"Rig observation has inconsistent timestamp or camera");
      const auto i=indices.at(frame.camera);
      try { item.seeds.emplace(i,matrix(estimate_target_pose(geometry,frame,initial[i].camera).T_cam_target)); }
      catch(const Error& error) {
        if(error.code!=3) throw;
        if(view.held_out) throw Error(3,"Held-out pose failed for "+frame.camera+": "+error.what());
        rejected.push_back({{"timestamp_ns",view.timestamp_ns},{"camera",frame.camera},{"reason",error.what()},{"held_out",view.held_out}});
      }
    }
    if(item.seeds.size()<2) {
      rejected.push_back({{"timestamp_ns",view.timestamp_ns},{"reason","fewer_than_two_valid_camera_poses"},{"held_out",view.held_out}});
      continue;
    }
    if(!view.held_out) for(auto a=item.seeds.begin();a!=item.seeds.end();++a) for(auto b=std::next(a);b!=item.seeds.end();++b)
      candidates[{a->first,b->first}].push_back(b->second*inverse(a->second));
    working.push_back(std::move(item));
  }
  std::vector<std::vector<std::pair<std::size_t,Matrix>>> graph(count);
  for(const auto& [pair,values]:candidates) if(values.size()>=6) {
    const Matrix relative=medoid(values);
    graph[pair.first].push_back({pair.second,relative});
    graph[pair.second].push_back({pair.first,inverse(relative)});
    edges.push_back({{"from",initial[pair.first].camera.id},{"to",initial[pair.second].camera.id},
      {"shared_training_poses",values.size()},{"initial_T_to_from",array(relative)}});
  }
  std::vector<std::optional<Matrix>> transforms(count);
  transforms[0]=Matrix::Identity(); // Declared reference gauge, not a calibration substitute.
  std::vector<std::size_t> queue{0};
  for(std::size_t q=0;q<queue.size();++q) for(const auto& [child,relative]:graph[queue[q]]) if(!transforms[child]) {
    transforms[child]=relative*(*transforms[queue[q]]); queue.push_back(child);
  }
  if(queue.size()!=count) throw Error(3,"Disconnected camera graph: every camera needs a path with at least six shared training poses per edge");
  std::vector<Pose> camera_poses(count);
  for(std::size_t i=0;i<count;++i) camera_poses[i]=pack(*transforms[i]);
  std::size_t training_views=0,held_out_views=0;
  std::vector<Eigen::Vector3d> normals;
  for(auto& item:working) {
    const auto& anchor=*item.seeds.begin();
    const Matrix target=inverse(*transforms[anchor.first])*anchor.second;
    item.target=pack(target);
    if(item.source->held_out) ++held_out_views;
    else { ++training_views; normals.push_back(target.block<3,1>(0,2)); }
  }
  if(training_views<8 || held_out_views<2) throw Error(3,"Rig needs eight training and two held-out synchronized views");
  double spread=0;
  for(const auto& a:normals) for(const auto& b:normals) spread=std::max(spread,std::acos(std::clamp(a.dot(b),-1.,1.)));
  if(spread<.08) throw Error(3,"Insufficient rig target tilt diversity");
  ceres::Problem problem;
  for(std::size_t i=0;i<count;++i) {
    problem.AddParameterBlock(camera_poses[i].data(),6);
    problem.AddParameterBlock(optics[i].data(),8);
    problem.SetParameterLowerBound(optics[i].data(),0,1.);
    problem.SetParameterLowerBound(optics[i].data(),1,1.);
  }
  problem.SetParameterBlockConstant(camera_poses[0].data());
  for(auto& item:working) if(!item.source->held_out) {
    for(const auto& frame:item.source->cameras) {
      const auto i=indices.at(frame.camera);
      if(!item.seeds.contains(i)) continue;
      for(const auto& corner:frame.corners) {
        const auto& point=geometry.tags()[corner.point_id/4][corner.point_id%4].position_m;
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<RigReprojection,2,6,6,8>(
          new RigReprojection{point,corner.pixel,kinds[i]}),new ceres::HuberLoss(2.),
          camera_poses[i].data(),item.target.data(),optics[i].data());
      }
    }
  }
  ceres::Solver::Options solver;
  solver.num_threads=static_cast<int>(options.threads); solver.max_num_iterations=100;
  solver.linear_solver_type=ceres::SPARSE_SCHUR;
  solver.sparse_linear_algebra_library_type=ceres::EIGEN_SPARSE;
  solver.function_tolerance=1e-10; solver.gradient_tolerance=1e-10; solver.parameter_tolerance=1e-10;
  solver.minimizer_progress_to_stdout=false;
  solver.linear_solver_ordering=std::make_shared<ceres::ParameterBlockOrdering>();
  for(auto& item:working) if(!item.source->held_out) solver.linear_solver_ordering->AddElementToGroup(item.target.data(),0);
  for(std::size_t i=0;i<count;++i) {
    solver.linear_solver_ordering->AddElementToGroup(camera_poses[i].data(),1);
    solver.linear_solver_ordering->AddElementToGroup(optics[i].data(),1);
  }
  CancelCallback callback(options); solver.callbacks.push_back(&callback);
  ceres::Solver::Summary summary; ceres::Solve(solver,&problem,&summary);
  check_cancelled(options);
  if(!summary.IsSolutionUsable() || summary.termination_type!=ceres::CONVERGENCE)
    throw Error(3,"Rig optimization did not converge: "+summary.BriefReport());
  RigCalibration result; result.reference_camera=initial[0].camera.id;
  for(std::size_t i=0;i<count;++i) {
    auto camera=initial[i].camera;
    camera.intrinsics.assign(optics[i].begin(),optics[i].begin()+4);
    camera.coefficients.assign(optics[i].begin()+4,optics[i].end());
    const auto transform=unpack(camera_poses[i]); const auto center=inverse(transform).block<3,1>(0,3).eval();
    result.cameras.push_back({camera,array(transform),{center.x(),center.y(),center.z()},0,0,0,0,{}});
  }
  std::vector<double> max_radius(count,0.);
  Json downweighted=Json::array(),pose_records=Json::array();
  std::vector<std::vector<double>> training_errors(count),validation_errors(count);
  for(auto& item:working) {
    check_cancelled(options);
    Matrix target;
    if(item.source->held_out) {
      // Fit the target pose using ONE camera only. Other cameras provide a cross-camera test.
      const auto anchor=item.seeds.begin()->first;
      const auto found=std::ranges::find(item.source->cameras,initial[anchor].camera.id,&FrameObservations::camera);
      const auto pose=estimate_target_pose(geometry,*found,result.cameras[anchor].optical);
      target=inverse(unpack(camera_poses[anchor]))*matrix(pose.T_cam_target);
    } else target=unpack(item.target);
    pose_records.push_back({{"timestamp_ns",item.source->timestamp_ns},{"held_out",item.source->held_out},
      {"T_reference_target",array(target)},{"held_out_anchor",item.source->held_out?Json(initial[item.seeds.begin()->first].camera.id):Json(nullptr)}});
    for(const auto& frame:item.source->cameras) {
      const auto i=indices.at(frame.camera);
      if(!item.seeds.contains(i)) continue;
      const Matrix transform=unpack(camera_poses[i])*target;
      const auto camera=make_camera_model(result.cameras[i].optical);
      for(const auto& corner:frame.corners) {
        const auto& p=geometry.tags()[corner.point_id/4][corner.point_id%4].position_m;
        const Eigen::Vector4d point=transform*Eigen::Vector4d(p[0],p[1],p[2],1.);
        const auto projection=camera.project({point.x(),point.y(),point.z()});
        if(!projection) throw Error(3,"Final rig projection is outside the optical domain");
        const double error=std::hypot(projection->pixel[0]-corner.pixel[0],projection->pixel[1]-corner.pixel[1]);
        max_radius[i]=std::max(max_radius[i],std::hypot(point.x(),point.y())/point.z());
        (item.source->held_out?validation_errors[i]:training_errors[i]).push_back(error);
        if(!item.source->held_out && error>2) downweighted.push_back({{"camera",frame.camera},
          {"timestamp_ns",frame.timestamp_ns},{"point_id",corner.point_id},{"residual_px",error},
          {"loss","Huber"},{"rho_prime",2/error}});
      }
    }
  }
  Json metrics=Json::array();
  for(std::size_t i=0;i<count;++i) {
    auto& camera=result.cameras[i];
    if(training_errors[i].empty() || validation_errors[i].empty()) throw Error(3,"Camera lacks training or held-out observations: "+camera.optical.id);
    auto rms=[](const std::vector<double>& errors){return std::sqrt(std::inner_product(errors.begin(),errors.end(),errors.begin(),0.)/static_cast<double>(errors.size()));};
    camera.training_rms_px=rms(training_errors[i]); camera.held_out_rms_px=rms(validation_errors[i]);
    camera.training_corners=training_errors[i].size(); camera.held_out_corners=validation_errors[i].size();
    if(camera.training_rms_px>5 || camera.held_out_rms_px>5) throw Error(3,"Rig reprojection RMS exceeds provisional 5 px limit: "+camera.optical.id);
    camera.optical_check=validate_optical_field(camera.optical,max_radius[i]);
    std::array<std::size_t,6> histogram{};
    for(double value:training_errors[i]) ++histogram[value<.25?0:value<.5?1:value<1?2:value<2?3:value<5?4:5];
    metrics.push_back({{"camera",camera.optical.id},{"training_residual_histogram",histogram},
      {"histogram_edges_px",{0,.25,.5,1,2,5}},{"last_bin","5 px and above"},
      {"max_observed_angle_rad",std::atan(max_radius[i])}});
  }
  result.diagnostics={{"policy_version",1},{"backend",std::string("Ceres ")+CERES_VERSION_STRING},
    {"solver",summary.BriefReport()},{"convergence",true},{"initial_cost",summary.initial_cost},{"final_cost",summary.final_cost},
    {"iterations",summary.iterations.size()},{"solver_seconds",summary.total_time_in_seconds},
    {"training_views",training_views},{"held_out_views",held_out_views},{"normal_spread_rad",spread},
    {"graph",edges},{"rejected_views",rejected},{"downweighted_points",downweighted},{"poses",pose_records},{"metrics",metrics},
    {"covariance",nullptr},{"observability","full_rank_not_assessed"},{"quality","provisional_not_certified"},
    {"held_out_protocol","timestamps excluded from monocular initialization and bundle adjustment; target pose fitted to one anchor camera"},
    {"temporal_model","exact common timestamps; global shutter; no offsets estimated"},
    {"gauge","first selected camera fixed; target metric geometry fixes scale"}};
  return result;
}
} // namespace calib
