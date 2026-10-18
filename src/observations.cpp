#include "calib/observations.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace calib {
AprilGridGeometry::AprilGridGeometry(const Target& target):target_(target) {
  if(target.rows<1 || target.cols<1 || target.rows>1000 || target.cols>1000 || target.first_id<0 ||
     !std::isfinite(target.size_m) || target.size_m<=0 ||
     !std::isfinite(target.spacing_ratio) || target.spacing_ratio<0)
    throw Error(2,"Invalid AprilGrid geometry");
  const auto count=std::int64_t(target.rows)*target.cols;
  if(std::int64_t(target.first_id)+count-1>std::numeric_limits<int>::max())
    throw Error(2,"AprilGrid ID range overflows int");
  const double pitch=target.size_m*(1+target.spacing_ratio);
  width_=(target.cols-1)*pitch+target.size_m;
  height_=(target.rows-1)*pitch+target.size_m;
  if(!std::isfinite(pitch) || !std::isfinite(width_) || !std::isfinite(height_))
    throw Error(2,"AprilGrid extent is not finite");
  tags_.resize(static_cast<std::size_t>(count));
  for(std::size_t index=0;index<tags_.size();++index) {
    const auto column=index%static_cast<std::size_t>(target.cols);
    const auto row=index/static_cast<std::size_t>(target.cols);
    const double x=static_cast<double>(column)*pitch,y=static_cast<double>(row)*pitch,s=target.size_m;
    if(!(x+s>x) || !(y+s>y)) throw Error(2,"AprilGrid corner separation is not representable in double");
    const std::array<std::array<double,3>,4> points{{{x,y,0},{x+s,y,0},{x+s,y+s,0},{x,y+s,0}}};
    for(unsigned corner=0;corner<4;++corner)
      tags_[index][corner]={static_cast<std::uint32_t>(4*index+corner),target.first_id+static_cast<int>(index),corner,points[corner]};
  }
}
bool AprilGridGeometry::contains(int tag_id) const {
  const auto index=std::int64_t(tag_id)-target_.first_id;
  return index>=0 && static_cast<std::uint64_t>(index)<tags_.size();
}
const std::array<GridCorner,4>& AprilGridGeometry::corners(int tag_id) const {
  if(!contains(tag_id)) throw Error(2,"Tag outside AprilGrid geometry: "+std::to_string(tag_id));
  return tags_[static_cast<std::size_t>(tag_id-target_.first_id)];
}

FrameObservations make_observations(const AprilGridGeometry& geometry,
                                    const FrameRef& frame,std::span<const Tag> tags) {
  if(frame.timestamp_ns<0 || !frame.image.width || !frame.image.height)
    throw Error(2,"Invalid frame metadata for observations");
  FrameObservations result{frame.camera,frame.timestamp_ns,{frame.image.width,frame.image.height},{},{}};
  result.corners.reserve(tags.size()*4);
  std::map<int,unsigned> occurrences;
  for(const auto& tag:tags) ++occurrences[tag.id];
  for(const auto& tag:tags) {
    std::string reason;
    if(!geometry.contains(tag.id)) reason="outside_grid_id_range";
    else if(occurrences[tag.id]>1) reason="duplicate_tag_id";
    else {
      for(const auto& point:tag.corners) {
        if(!std::isfinite(point[0]) || !std::isfinite(point[1])) { reason="nonfinite_corner"; break; }
        if(point[0]<0 || point[1]<0 || point[0]>=frame.image.width || point[1]>=frame.image.height)
          reason="corner_outside_image";
      }
      if(reason.empty()) for(unsigned k=0;k<4;++k) {
        const auto& a=tag.corners[k]; const auto& b=tag.corners[(k+1)%4]; const auto& c=tag.corners[(k+2)%4];
        const double cross=(b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]);
        // Canonical AprilTag winding is negative in pixel coordinates (Y down).
        if(!(cross<0)) { reason="invalid_corner_winding_or_degenerate"; break; }
      }
    }
    if(!reason.empty()) { result.rejected.push_back({tag.id,std::move(reason)}); continue; }
    for(unsigned k=0;k<4;++k) result.corners.push_back({geometry.corners(tag.id)[k].point_id,tag.corners[k]});
  }
  std::ranges::sort(result.corners,{},&CornerObservation::point_id);
  std::ranges::stable_sort(result.rejected,{},&RejectedObservation::tag_id);
  return result;
}

Json geometry_json(const AprilGridGeometry& geometry) {
  Json result{{"schema_version",1},{"units","m"},{"frame","target"},
    {"origin","p0 of tagStartId"},{"axes","+X increasing columns, +Y increasing rows, +Z = X cross Y"},
    {"layout","row_major_ids; all tags share canonical orientation"},
    {"point_id_rule","4 * (tag_id - tagStartId) + corner_index; not Kalibr's flattened grid index"},
    {"tag_size_definition","side between detected border corners, excluding any outer quiet zone"},
    {"width_m",geometry.width_m()},{"height_m",geometry.height_m()},
    {"family",geometry.target().family},{"border_bits",geometry.target().border_bits},
    {"points",Json::array()}};
  for(const auto& tag:geometry.tags()) for(const auto& corner:tag)
    result["points"].push_back({{"point_id",corner.point_id},{"tag_id",corner.tag_id},
      {"corner_index",corner.corner_index},{"position_m",corner.position_m}});
  return result;
}
Json observations_json(const FrameObservations& observations) {
  Json result{{"schema_version",1},{"camera",observations.camera},{"timestamp_ns",observations.timestamp_ns},
    {"resolution",observations.resolution},{"points",Json::array()},{"rejected_tags",Json::array()},
    {"status",observations.corners.empty()?"no_correspondences":"correspondences_available"}};
  for(const auto& corner:observations.corners)
    result["points"].push_back({{"point_id",corner.point_id},{"pixel",corner.pixel}});
  for(const auto& rejected:observations.rejected)
    result["rejected_tags"].push_back({{"id",rejected.tag_id},{"reason",rejected.reason}});
  return result;
}
} // namespace calib
