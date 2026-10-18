#include "calib/core.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <apriltag.h>
#include <tag36h11.h>
#include <tagStandard41h12.h>

namespace calib {
struct Detector::Impl {
  std::unique_ptr<apriltag_family_t,void(*)(apriltag_family_t*)> family;
  std::unique_ptr<apriltag_detector_t,void(*)(apriltag_detector_t*)> detector;
  explicit Impl(const std::string& name,double decimate,int border_bits)
    : family(name=="tag36h11"?tag36h11_create():tagStandard41h12_create(),
             name=="tag36h11"?tag36h11_destroy:tagStandard41h12_destroy),
      detector(apriltag_detector_create(),apriltag_detector_destroy) {
    if(!family || !detector) throw std::bad_alloc();
    if(!std::isfinite(decimate) || decimate<1 || decimate>4) throw Error(2,"quad decimation must be in [1,4]");
    if(border_bits==2) {
      // Extend the solid black border by one cell on every side; keep the codebook unchanged.
      family->width_at_border+=2;
      family->total_width+=2;
      for(std::uint32_t bit=0;bit<family->nbits;++bit) { ++family->bit_x[bit]; ++family->bit_y[bit]; }
    }
    detector->nthreads=1; // Frame-level parallelism owns the global thread budget.
    detector->quad_decimate=static_cast<float>(decimate);
    detector->refine_edges=true;
    apriltag_detector_add_family_bits(detector.get(),family.get(),1);
  }
};
Detector::Detector(const std::string& family,double decimate,int border_bits) {
  if(family!="tag36h11" && family!="tagStandard41h12") throw Error(8,"Unsupported tag family: "+family);
  if(border_bits!=1 && !(border_bits==2 && family=="tag36h11")) throw Error(8,"Unsupported family/border combination");
  impl_=std::make_unique<Impl>(family,decimate,border_bits);
}
Detector::Detector(const Target& target,double decimate):Detector(target.family,decimate,target.border_bits) {
  if(target.rows<1 || target.cols<1 || target.rows>1000 || target.cols>1000 || target.first_id<0)
    throw Error(2,"Invalid target ID range");
  if(std::uint64_t(target.first_id)+std::uint64_t(target.rows)*std::uint64_t(target.cols)>impl_->family->ncodes)
    throw Error(2,"AprilGrid IDs exceed the chosen tag family");
}
Detector::~Detector()=default;
std::vector<Tag> Detector::detect(GrayImage& image) {
  if(!image.info.width || !image.info.height || image.info.width>=32768 || image.info.height>=32768 ||
      image.pixels.size()!=std::size_t(image.info.width)*image.info.height)
    throw Error(2,"Invalid grayscale image buffer or dimensions outside AprilTag range 1..32767");
  // Upstream thresholding uses 4x4 tiles; smaller images cannot contain a tag.
  if(image.info.width<4 || image.info.height<4) return {};
  image_u8_t input{static_cast<int>(image.info.width),static_cast<int>(image.info.height),static_cast<int>(image.info.width),image.pixels.data()};
  std::unique_ptr<zarray_t,void(*)(zarray_t*)> detections(apriltag_detector_detect(impl_->detector.get(),&input),apriltag_detections_destroy);
  if(!detections) throw Error(4,"AprilTag detection failed");
  std::vector<Tag> tags;
  std::set<int> seen;
  for(int i=0;i<zarray_size(detections.get());++i) {
    apriltag_detection_t* detection=nullptr;
    zarray_get(detections.get(),i,&detection);
    if(!seen.insert(detection->id).second) throw Error(2,"Duplicate AprilTag ID in a frame: "+std::to_string(detection->id));
    Tag tag{detection->id,detection->hamming,detection->decision_margin,{}};
    for(std::size_t k=0;k<4;++k) for(std::size_t axis=0;axis<2;++axis) {
      tag.corners[k][axis]=detection->p[k][axis];
      if(!std::isfinite(tag.corners[k][axis])) throw Error(4,"Non-finite AprilTag corner");
    }
    tags.push_back(tag);
  }
  std::ranges::sort(tags,{},&Tag::id);
  return tags;
}
Json tags_json(const FrameRef& frame,const std::vector<Tag>& tags) {
  Json result{{"camera",frame.camera},{"timestamp_ns",frame.timestamp_ns},
              {"resolution",{frame.image.width,frame.image.height}},{"tags",Json::array()}};
  for(const auto& tag:tags) result["tags"].push_back({{"id",tag.id},{"hamming",tag.hamming},
      {"decision_margin",tag.margin},{"corners_px",tag.corners}});
  return result;
}
} // namespace calib
