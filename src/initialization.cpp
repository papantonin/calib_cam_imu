#include "calib/initialization.hpp"
#include <opencv2/calib.hpp>
#include <opencv2/geometry.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <numbers>
#include <mutex>

namespace calib {
InitialViewSampler::InitialViewSampler(std::size_t frames,std::size_t max_views)
    : frames_(frames),bins_(std::min(frames,max_views)) {
  if(!frames || max_views<10 || max_views>200) throw Error(2,"Invalid initialization view budget");
}
void InitialViewSampler::consider(FrameObservations frame) {
  if(seen_>=frames_) throw Error(2,"More frames than declared to initialization sampler");
  const auto bin=seen_++*bins_.size()/frames_;
  if(frame.corners.size()>=12 && (!bins_[bin] || frame.corners.size()>bins_[bin]->corners.size()))
    bins_[bin]=std::move(frame);
}
std::vector<FrameObservations> InitialViewSampler::selected() const {
  std::vector<FrameObservations> frames;
  for(const auto& bin:bins_) if(bin) frames.push_back(*bin);
  return frames;
}
namespace {
void configure_backend() {
  static std::once_flag configured;
  std::call_once(configured,[]{cv::setNumThreads(1);});
}
struct View {
  std::int64_t timestamp{};
  std::vector<cv::Point3d> object;
  std::vector<cv::Point2d> image;
  std::vector<std::uint32_t> ids;
};
struct Fit {
  cv::Mat K,D;
  std::vector<cv::Mat> rotations,translations;
  double rms{};
};
void check_cancelled(const std::function<bool()>& cancelled) {
  if(cancelled && cancelled()) throw Error(130,"Initialization interrupted between solver calls");
}
CameraModel model(const Fit& fit,DistortionModel kind) {
  return CameraModel(kind,{fit.K.at<double>(0,0),fit.K.at<double>(1,1),fit.K.at<double>(0,2),fit.K.at<double>(1,2),
    fit.D.at<double>(0),fit.D.at<double>(1),fit.D.at<double>(2),fit.D.at<double>(3)});
}
Fit fit_views(const std::vector<View>& views,cv::Size size,DistortionModel kind,double focal,const Fit* initial=nullptr) {
  Fit fit;
  fit.K=cv::Mat::eye(3,3,CV_64F);
  fit.K.at<double>(0,0)=focal; fit.K.at<double>(1,1)=focal;
  fit.K.at<double>(0,2)=(size.width-1)*.5; fit.K.at<double>(1,2)=(size.height-1)*.5;
  fit.D=cv::Mat::zeros(kind==DistortionModel::equidistant?4:5,1,CV_64F);
  if(initial) { fit.K=initial->K.clone(); fit.D=initial->D.clone(); }
  const auto criteria=cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,100,1e-9);
  if(kind==DistortionModel::equidistant) {
    std::vector<std::vector<cv::Point3d>> objects;
    std::vector<std::vector<cv::Point2d>> images;
    for(const auto& view:views) { objects.push_back(view.object); images.push_back(view.image); }
    fit.rms=cv::fisheye::calibrate(objects,images,size,fit.K,fit.D,fit.rotations,fit.translations,
      cv::CALIB_USE_INTRINSIC_GUESS|cv::CALIB_RECOMPUTE_EXTRINSIC|cv::CALIB_FIX_SKEW|cv::CALIB_CHECK_COND,criteria);
  } else {
    // OpenCV's pinhole entry point requires float observations, while its solve
    // and our independent reprojection checks use double precision.
    std::vector<std::vector<cv::Point3f>> objects;
    std::vector<std::vector<cv::Point2f>> images;
    for(const auto& view:views) {
      objects.emplace_back(view.object.begin(),view.object.end());
      images.emplace_back(view.image.begin(),view.image.end());
    }
    fit.rms=cv::calibrateCamera(objects,images,size,fit.K,fit.D,fit.rotations,fit.translations,
      cv::CALIB_FIX_K3|(initial?cv::CALIB_USE_INTRINSIC_GUESS:0),criteria);
  }
  (void)model(fit,kind);
  if(!std::isfinite(fit.rms)) throw Error(3,"Nonfinite initialization residual");
  return fit;
}
std::vector<double> residuals(const View& view,const cv::Mat& r,const cv::Mat& t,const CameraModel& camera,
                              double* max_radius=nullptr) {
  cv::Mat rotation; cv::Rodrigues(r,rotation);
  std::vector<double> errors;
  for(std::size_t i=0;i<view.object.size();++i) {
    const auto& p=view.object[i]; std::array<double,3> point;
    for(int row=0;row<3;++row) point[static_cast<std::size_t>(row)]=rotation.at<double>(row,0)*p.x+
      rotation.at<double>(row,1)*p.y+rotation.at<double>(row,2)*p.z+t.at<double>(row);
    const auto projected=camera.project(point);
    if(!projected) throw Error(3,"Initial target pose projects outside the forward camera domain");
    errors.push_back(std::hypot(projected->pixel[0]-view.image[i].x,projected->pixel[1]-view.image[i].y));
    if(max_radius) *max_radius=std::max(*max_radius,std::hypot(point[0],point[1])/point[2]);
  }
  return errors;
}
double squared_sum(const std::vector<double>& values) {
  return std::inner_product(values.begin(),values.end(),values.begin(),0.);
}
double median(std::vector<double> values) {
  const auto middle=values.begin()+static_cast<std::ptrdiff_t>(values.size()/2);
  std::nth_element(values.begin(),middle,values.end()); return *middle;
}
InitialPose pose_result(const View& view,const cv::Mat& r,const cv::Mat& t,const std::vector<double>& errors,bool held_out) {
  InitialPose result;
  result.timestamp_ns=view.timestamp; result.corners=view.object.size(); result.held_out=held_out;
  result.rms_px=std::sqrt(squared_sum(errors)/static_cast<double>(errors.size()));
  cv::Mat rotation; cv::Rodrigues(r,rotation);
  for(int row=0;row<3;++row) {
    for(int col=0;col<3;++col) result.T_cam_target[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)]=rotation.at<double>(row,col);
    result.T_cam_target[static_cast<std::size_t>(row)][3]=t.at<double>(row);
  }
  result.T_cam_target[3][3]=1; return result;
}
double check_distortion(const CameraModel& camera,double radius) {
  double minimum=std::numeric_limits<double>::infinity();
  const auto& p=camera.parameters();
  for(unsigned ri=0;ri<=64;++ri) for(unsigned ai=0;ai<32;++ai) {
    const double r=radius*ri/64,angle=2*std::numbers::pi*ai/32;
    const auto value=camera.project({r*std::cos(angle),r*std::sin(angle),1});
    if(!value) throw Error(3,"Invalid sampled distortion domain");
    const double a=value->point_jacobian[0][0]/p[0], b=value->point_jacobian[0][1]/p[0];
    const double d=value->point_jacobian[1][1]/p[1];
    const double eigen=.5*(a+d-std::hypot(a-d,2*b));
    if(!(eigen>0)) throw Error(3,"Distortion folds within the sampled observed angular field");
    minimum=std::min(minimum,eigen);
  }
  return minimum;
}
double check_inverse(const CameraModel& camera,const Fit& fit,double radius) {
  std::vector<cv::Point2d> pixels,rays,inverse;
  for(unsigned ri=0;ri<=64;++ri) for(unsigned ai=0;ai<32;++ai) {
    const double r=radius*ri/64,angle=2*std::numbers::pi*ai/32;
    rays.emplace_back(r*std::cos(angle),r*std::sin(angle));
    const auto projected=camera.project({rays.back().x,rays.back().y,1}).value();
    pixels.emplace_back(projected.pixel[0],projected.pixel[1]);
  }
  const cv::TermCriteria criteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,100,1e-12);
  try {
    if(camera.distortion()==DistortionModel::equidistant)
      cv::fisheye::undistortPoints(pixels,inverse,fit.K,fit.D,cv::noArray(),cv::noArray(),criteria);
    else cv::undistortPoints(pixels,inverse,fit.K,fit.D,cv::noArray(),cv::noArray(),criteria);
  } catch(const cv::Exception& error) { throw Error(3,"Distortion inversion failed: "+std::string(error.what())); }
  if(inverse.size()!=rays.size()) throw Error(3,"Incomplete distortion inversion");
  double maximum=0;
  for(std::size_t i=0;i<rays.size();++i) {
    const double error=std::hypot(inverse[i].x-rays[i].x,inverse[i].y-rays[i].y);
    if(!std::isfinite(error) || error>1e-7*(1+std::hypot(rays[i].x,rays[i].y)))
      throw Error(3,"Distortion inverse fails sampled ray roundtrip");
    maximum=std::max(maximum,error);
  }
  return maximum;
}
}

InitialCalibration initialize_camera(const AprilGridGeometry& geometry,std::span<const FrameObservations> frames,
                                      DistortionModel kind,const std::function<bool()>& cancelled) {
  if(frames.size()<10 || frames.size()>200) throw Error(3,"Initialization requires 10 to 200 eligible views per camera");
  configure_backend();
  const auto size=frames.front().resolution;
  if(!size[0] || !size[1] || size[0]>32768 || size[1]>32768) throw Error(2,"Invalid initialization resolution");
  InitialCalibration result; result.selected_views=frames.size();
  std::vector<View> training,validation;
  std::int64_t previous=-1;
  for(std::size_t index=0;index<frames.size();++index) {
    const auto& frame=frames[index];
    if(frame.camera!=frames.front().camera || frame.resolution!=size || frame.timestamp_ns<=previous)
      throw Error(2,"Initialization views must have one camera, resolution and increasing timestamps");
    previous=frame.timestamp_ns;
    if(frame.corners.size()<12) throw Error(3,"Initialization views require at least twelve corners");
    View view; view.timestamp=frame.timestamp_ns;
    std::set<std::uint32_t> ids;
    for(const auto& corner:frame.corners) {
      if(corner.point_id>=geometry.tags().size()*4 || !ids.insert(corner.point_id).second)
        throw Error(2,"Unknown or duplicate initialization point ID");
      if(!std::isfinite(corner.pixel[0]) || !std::isfinite(corner.pixel[1]) || corner.pixel[0]<0 || corner.pixel[1]<0 ||
          corner.pixel[0]>=size[0] || corner.pixel[1]>=size[1]) throw Error(2,"Initialization pixel outside image");
      const auto& p=geometry.tags()[corner.point_id/4][corner.point_id%4].position_m;
      view.object.emplace_back(p[0],p[1],p[2]); view.image.emplace_back(corner.pixel[0],corner.pixel[1]);
      view.ids.push_back(corner.point_id);
    }
    (index%5==4?validation:training).push_back(std::move(view));
  }
  Fit best; bool found=false;
  for(const double fraction:{.3,.5,.8}) {
    check_cancelled(cancelled);
    try {
      auto fit=fit_views(training,{static_cast<int>(size[0]),static_cast<int>(size[1])},kind,std::max(size[0],size[1])*fraction);
      double radius=0; const auto camera=model(fit,kind);
      for(std::size_t i=0;i<training.size();++i) (void)residuals(training[i],fit.rotations[i],fit.translations[i],camera,&radius);
      (void)check_distortion(camera,radius);
      if(!found || fit.rms<best.rms) { best=std::move(fit); found=true; }
    } catch(const std::exception& error) { result.failed_attempts.push_back(error.what()); }
    if(kind==DistortionModel::radtan) break; // Zhang seed, independent of focal trial.
  }
  if(!found) throw Error(3,"All intrinsic initialization attempts failed: "+result.failed_attempts.front());
  check_cancelled(cancelled);
  std::vector<std::vector<double>> errors;
  std::vector<double> all_errors;
  for(std::size_t i=0;i<training.size();++i) {
    errors.push_back(residuals(training[i],best.rotations[i],best.translations[i],model(best,kind)));
    all_errors.insert(all_errors.end(),errors.back().begin(),errors.back().end());
  }
  const double center=median(all_errors);
  for(auto& value:all_errors) value=std::abs(value-center);
  const double threshold=std::max(3.,center+4.5*1.4826*median(all_errors));
  std::vector<View> filtered;
  for(std::size_t i=0;i<training.size();++i) {
    View view; view.timestamp=training[i].timestamp;
    for(std::size_t j=0;j<training[i].ids.size();++j) {
      if(errors[i][j]>threshold) result.rejected.push_back({view.timestamp,training[i].ids[j],"initial_reprojection_outlier",errors[i][j]});
      else { view.object.push_back(training[i].object[j]); view.image.push_back(training[i].image[j]); view.ids.push_back(training[i].ids[j]); }
    }
    if(view.ids.size()>=12) filtered.push_back(std::move(view));
    else for(auto id:view.ids) {
      const auto it=std::ranges::find(training[i].ids,id);
      result.rejected.push_back({view.timestamp,id,"view_has_fewer_than_twelve_remaining_corners",errors[i][static_cast<std::size_t>(it-training[i].ids.begin())]});
    }
  }
  if(filtered.size()<8) throw Error(3,"Too few views after initial reprojection filtering");
  if(!result.rejected.empty()) {
    check_cancelled(cancelled);
    try { best=fit_views(filtered,{static_cast<int>(size[0]),static_cast<int>(size[1])},kind,0,&best); }
    catch(const cv::Exception& error) { throw Error(3,"Initialization refit failed: "+std::string(error.what())); }
  }
  training=std::move(filtered);
  const auto camera=model(best,kind);
  const auto& p=camera.parameters();
  result.camera={frames.front().camera,"pinhole",kind==DistortionModel::radtan?"radtan":"equidistant",
    {p[0],p[1],p[2],p[3]},{p[4],p[5],p[6],p[7]},size[0],size[1],0,false};
  double sum=0,radius=0; std::size_t count=0;
  for(std::size_t i=0;i<training.size();++i) {
    const auto values=residuals(training[i],best.rotations[i],best.translations[i],camera,&radius);
    sum+=squared_sum(values); count+=values.size();
    result.poses.push_back(pose_result(training[i],best.rotations[i],best.translations[i],values,false));
  }
  result.training_rms_px=std::sqrt(sum/static_cast<double>(count));
  for(const auto& a:result.poses) for(const auto& b:result.poses) {
    double dot=0; for(unsigned i=0;i<3;++i) dot+=a.T_cam_target[i][2]*b.T_cam_target[i][2];
    result.normal_spread_rad=std::max(result.normal_spread_rad,std::acos(std::clamp(dot,-1.,1.)));
  }
  if(result.normal_spread_rad<.08) throw Error(3,"Insufficient target tilt diversity (normal spread below 0.08 rad)");
  sum=0; count=0;
  for(const auto& view:validation) {
    check_cancelled(cancelled);
    cv::Mat rotation,translation;
    try {
      bool ok;
      if(kind==DistortionModel::equidistant) {
        std::vector<cv::Point2d> undistorted;
        cv::fisheye::undistortPoints(view.image,undistorted,best.K,best.D);
        ok=cv::solvePnP(view.object,undistorted,cv::Mat::eye(3,3,CV_64F),cv::noArray(),rotation,translation);
      } else ok=cv::solvePnP(view.object,view.image,best.K,best.D,rotation,translation);
      if(!ok) throw Error(3,"Held-out target pose initialization failed");
    } catch(const cv::Exception& error) { throw Error(3,"Held-out pose failed: "+std::string(error.what())); }
    const auto values=residuals(view,rotation,translation,camera,&radius);
    sum+=squared_sum(values); count+=values.size();
    result.poses.push_back(pose_result(view,rotation,translation,values,true));
  }
  result.held_out_rms_px=std::sqrt(sum/static_cast<double>(count));
  result.max_observed_angle_rad=std::atan(radius);
  result.min_sampled_distortion_eigenvalue=check_distortion(camera,radius);
  result.max_inverse_roundtrip_ray_error=check_inverse(camera,best,radius);
  if(result.training_rms_px>5 || result.held_out_rms_px>5)
    throw Error(3,"Initial reprojection RMS exceeds the provisional 5 px acceptance limit");
  std::ranges::sort(result.poses,{},&InitialPose::timestamp_ns);
  return result;
}
InitialPose estimate_target_pose(const AprilGridGeometry& geometry,const FrameObservations& frame,const CameraCalibration& camera) {
  configure_backend();
  if(frame.corners.size()<12 || frame.camera!=camera.id || frame.resolution!=std::array<std::uint32_t,2>{camera.width,camera.height})
    throw Error(3,"Target pose requires twelve corners and matching camera/resolution");
  const auto optical=make_camera_model(camera);
  cv::Mat K=cv::Mat::eye(3,3,CV_64F),D(4,1,CV_64F);
  K.at<double>(0,0)=camera.intrinsics[0]; K.at<double>(1,1)=camera.intrinsics[1];
  K.at<double>(0,2)=camera.intrinsics[2]; K.at<double>(1,2)=camera.intrinsics[3];
  for(int i=0;i<4;++i) D.at<double>(i)=camera.coefficients[static_cast<std::size_t>(i)];
  View view; view.timestamp=frame.timestamp_ns;
  std::set<std::uint32_t> ids;
  for(const auto& corner:frame.corners) {
    if(corner.point_id>=geometry.tags().size()*4 || !ids.insert(corner.point_id).second)
      throw Error(2,"Unknown or duplicate pose point ID");
    if(!std::isfinite(corner.pixel[0]) || !std::isfinite(corner.pixel[1]) || corner.pixel[0]<0 || corner.pixel[1]<0 ||
        corner.pixel[0]>=camera.width || corner.pixel[1]>=camera.height) throw Error(2,"Invalid pose observation pixel");
    const auto& p=geometry.tags()[corner.point_id/4][corner.point_id%4].position_m;
    view.object.emplace_back(p[0],p[1],p[2]); view.image.emplace_back(corner.pixel[0],corner.pixel[1]);
  }
  cv::Mat r,t;
  try {
    std::vector<cv::Point2d> normalized;
    const cv::TermCriteria criteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,100,1e-12);
    if(camera.distortion=="equidistant") cv::fisheye::undistortPoints(view.image,normalized,K,D,cv::noArray(),cv::noArray(),criteria);
    else cv::undistortPoints(view.image,normalized,K,D,cv::noArray(),cv::noArray(),criteria);
    if(!cv::solvePnP(view.object,normalized,cv::Mat::eye(3,3,CV_64F),cv::noArray(),r,t))
      throw Error(3,"Target pose solve failed");
    const auto errors=residuals(view,r,t,optical);
    auto pose=pose_result(view,r,t,errors,false);
    if(pose.rms_px>10) throw Error(3,"Target pose seed exceeds 10 px RMS");
    return pose;
  } catch(const cv::Exception& error) { throw Error(3,"Target pose solve failed: "+std::string(error.what())); }
}
OpticalFieldCheck validate_optical_field(const CameraCalibration& camera,double radius) {
  if(!std::isfinite(radius) || radius<0) throw Error(3,"Invalid observed optical radius");
  const auto optical=make_camera_model(camera);
  Fit fit; fit.K=cv::Mat::eye(3,3,CV_64F); fit.D=cv::Mat(4,1,CV_64F);
  fit.K.at<double>(0,0)=camera.intrinsics[0]; fit.K.at<double>(1,1)=camera.intrinsics[1];
  fit.K.at<double>(0,2)=camera.intrinsics[2]; fit.K.at<double>(1,2)=camera.intrinsics[3];
  for(int i=0;i<4;++i) fit.D.at<double>(i)=camera.coefficients[static_cast<std::size_t>(i)];
  return {check_distortion(optical,radius),check_inverse(optical,fit,radius)};
}
} // namespace calib
