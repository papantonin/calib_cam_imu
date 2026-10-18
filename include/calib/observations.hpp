#pragma once
#include "calib/core.hpp"
#include <span>

namespace calib {
struct GridCorner {
  std::uint32_t point_id{};
  int tag_id{};
  unsigned corner_index{};
  std::array<double,3> position_m{};
};

// Origin: p0 of the first tag. +X follows columns, +Y follows rows, Z = X cross Y.
// Corner order p0,p1,p2,p3: (0,0), (size,0), (size,size), (0,size).
class AprilGridGeometry {
 public:
  explicit AprilGridGeometry(const Target& target);
  const std::array<GridCorner,4>& corners(int tag_id) const;
  bool contains(int tag_id) const;
  const Target& target() const { return target_; }
  const std::vector<std::array<GridCorner,4>>& tags() const { return tags_; }
  double width_m() const { return width_; }
  double height_m() const { return height_; }
 private:
  Target target_;
  std::vector<std::array<GridCorner,4>> tags_;
  double width_{},height_{};
};

struct CornerObservation {
  std::uint32_t point_id{};
  std::array<double,2> pixel{};
};
struct RejectedObservation { int tag_id{}; std::string reason; };
struct FrameObservations {
  std::string camera;
  std::int64_t timestamp_ns{};
  std::array<std::uint32_t,2> resolution{};
  std::vector<CornerObservation> corners;
  std::vector<RejectedObservation> rejected;
};
FrameObservations make_observations(const AprilGridGeometry& geometry,
                                    const FrameRef& frame,std::span<const Tag> tags);
Json geometry_json(const AprilGridGeometry& geometry);
Json observations_json(const FrameObservations& observations);
} // namespace calib
