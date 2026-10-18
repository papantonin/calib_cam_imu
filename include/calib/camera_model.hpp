#pragma once
#include <array>
#include <expected>

namespace calib {
enum class DistortionModel { radtan, equidistant };
enum class ProjectionError { nonfinite_point, nonpositive_depth, numerical_overflow };

struct Projection {
  std::array<double,2> pixel;
  // Row-major derivatives: rows u,v; columns X,Y,Z in the camera frame.
  std::array<std::array<double,3>,2> point_jacobian;
  // Columns fx,fy,cx,cy,k1,k2,p1,p2 (radtan), or fx,fy,cx,cy,k1,k2,k3,k4.
  std::array<std::array<double,8>,2> parameter_jacobian;
};

// X right, Y down, Z forward. Zero skew. No image clipping or shutter timing here.
// Polynomial evaluation does not certify distortion invertibility or lens validity.
class CameraModel {
 public:
  using Parameters = std::array<double,8>;
  // Throws invalid_argument for nonfinite parameters, nonpositive focal lengths
  // or an unknown enum. Instances are immutable and safe for concurrent reads.
  CameraModel(DistortionModel distortion, Parameters parameters);
  [[nodiscard]] DistortionModel distortion() const noexcept { return distortion_; }
  [[nodiscard]] const Parameters& parameters() const noexcept { return parameters_; }
  // Both models currently require Z > 0. Nonfinite results/Jacobians are errors.
  [[nodiscard]] std::expected<Projection,ProjectionError>
  project(const std::array<double,3>& point_camera) const noexcept;
 private:
  DistortionModel distortion_;
  Parameters parameters_;
};
} // namespace calib
