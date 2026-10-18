#pragma once
#include "calib/camera_model.hpp"
#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
#include <nlohmann/json.hpp>

namespace calib {
using Json = nlohmann::json;
namespace fs = std::filesystem;
struct Error : std::runtime_error {
  int code;
  Error(int status, const std::string& message) : std::runtime_error(message), code(status) {}
};
std::int64_t parse_timestamp(std::string_view value, std::string_view context);
double relative_seconds(std::int64_t timestamp, std::int64_t origin);
struct ImageInfo { std::uint32_t width{}, height{}; unsigned channels{}, bit_depth{}; };
struct GrayImage { ImageInfo info; std::vector<std::uint8_t> pixels; };
ImageInfo probe_png(const fs::path& path);
GrayImage decode_png(const fs::path& path, std::size_t max_pixels);
struct FrameRef { std::string camera; std::int64_t timestamp_ns{}; fs::path path; ImageInfo image; };
struct CameraStream { std::string id; std::vector<FrameRef> frames; };
struct ImuSample { std::int64_t timestamp_ns{}; std::array<double, 6> values{}; };
struct Dataset { fs::path root; std::vector<CameraStream> cameras; Json imu; };
// Other formats can implement this contract without changing image processing.
class DatasetReader {
 public:
  virtual ~DatasetReader() = default;
  virtual Dataset index(const std::vector<std::string>& selected, bool require_imu) const = 0;
  virtual GrayImage decode(const FrameRef&, std::size_t max_pixels) const = 0;
};
class NativeDirectoryReader final : public DatasetReader {
 public:
  explicit NativeDirectoryReader(fs::path root) : root_(std::move(root)) {}
  Dataset index(const std::vector<std::string>& selected, bool require_imu) const override;
  GrayImage decode(const FrameRef& frame, std::size_t max_pixels) const override;
 private:
  fs::path root_;
};
Json read_imu(const fs::path& path, const std::function<void(const ImuSample&)>& consume = {});
Json dataset_summary(const Dataset& dataset);
struct CameraCalibration {
  std::string id, model, distortion;
  std::vector<double> intrinsics, coefficients;
  std::uint32_t width{}, height{};
  double line_delay_s{};
  bool has_line_delay{};
};
std::vector<CameraCalibration> read_camchain(const fs::path& path);
CameraModel make_camera_model(const CameraCalibration& calibration);
// Kalibr convention: row 0 reference, signed seconds per image row.
double corner_time(std::int64_t timestamp, std::int64_t origin,
                   double row, double line_delay_s, double offset_s);
struct Target {
  std::string family;
  int rows{}, cols{}, first_id{};
  double size_m{}, spacing_ratio{};
  int border_bits{1};
};
Target read_target(const fs::path& path);
struct Tag {
  int id{}, hamming{};
  double margin{};
  std::array<std::array<double, 2>, 4> corners{};
};
class Detector {
 public:
  explicit Detector(const Target& target, double decimate = 1.0);
  explicit Detector(const std::string& family, double decimate = 1.0, int border_bits = 1);
  ~Detector();
  Detector(const Detector&) = delete;
  Detector& operator=(const Detector&) = delete;
  std::vector<Tag> detect(GrayImage& image);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
Json tags_json(const FrameRef& frame, const std::vector<Tag>& tags);
} // namespace calib
