#include "calib/core.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <span>
#include <png.h>
#include <zlib.h>

namespace calib {
namespace {
std::string trim(std::string_view s) {
  const auto first = s.find_first_not_of(" \t\r");
  return first == s.npos ? "" : std::string(s.substr(first, s.find_last_not_of(" \t\r") - first + 1));
}
std::vector<std::string> split(std::string_view line) {
  std::vector<std::string> cells;
  std::size_t start = 0;
  while (true) {
    const auto end = line.find(',', start);
    cells.push_back(trim(line.substr(start, end == line.npos ? line.size() - start : end - start)));
    if (end == line.npos) return cells;
    start = end + 1;
  }
}
std::uint32_t big_endian(const unsigned char* p) {
  return (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) |
         (std::uint32_t(p[2]) << 8) | std::uint32_t(p[3]);
}
struct PngHandle {
  png_image image{};
  PngHandle() { image.version = PNG_IMAGE_VERSION; }
  ~PngHandle() { png_image_free(&image); }
};
struct RawPngHandle {
  png_structp reader{};
  png_infop info{};
  char error[256]{};
  ~RawPngHandle() { if(reader) png_destroy_read_struct(&reader, &info, nullptr); }
};
void raw_png_error(png_structp reader, png_const_charp message) {
  auto* handle=static_cast<RawPngHandle*>(png_get_error_ptr(reader));
  std::strncpy(handle->error,message,sizeof(handle->error)-1);
  png_longjmp(reader,1);
}
GrayImage decode_mono16(const fs::path& path,const ImageInfo& shape) {
  const auto close_file=[](FILE* file) { std::fclose(file); };
  std::unique_ptr<FILE,decltype(close_file)> file(std::fopen(path.c_str(),"rb"),close_file);
  if(!file) throw Error(5,"Cannot read PNG: "+path.string());
  auto handle=std::make_unique<RawPngHandle>();
  handle->reader=png_create_read_struct(PNG_LIBPNG_VER_STRING,handle.get(),raw_png_error,raw_png_error);
  if(!handle->reader) throw std::bad_alloc();
  handle->info=png_create_info_struct(handle->reader);
  if(!handle->info) throw std::bad_alloc();
  GrayImage result{shape,std::vector<std::uint8_t>(std::size_t(shape.width)*shape.height)};
  std::vector<png_bytep> rows(shape.height);
  for(std::size_t y=0;y<rows.size();++y) rows[y]=result.pixels.data()+y*shape.width;
  // All C++ owners are constructed before setjmp; callbacks only jump through libpng C frames.
  if(setjmp(png_jmpbuf(handle->reader))) throw Error(2,"PNG decode failed: "+path.string()+": "+handle->error);
  png_init_io(handle->reader,file.get());
  png_set_crc_action(handle->reader,PNG_CRC_ERROR_QUIT,PNG_CRC_ERROR_QUIT);
  png_read_info(handle->reader,handle->info);
  if(png_get_image_width(handle->reader,handle->info)!=shape.width ||
     png_get_image_height(handle->reader,handle->info)!=shape.height ||
     png_get_bit_depth(handle->reader,handle->info)!=16 ||
     png_get_color_type(handle->reader,handle->info)!=PNG_COLOR_TYPE_GRAY)
    png_error(handle->reader,"PNG changed after header inspection");
  // Exact high byte (floor(raw/256)); no gamma transform or per-frame normalization.
  png_set_strip_16(handle->reader);
  png_set_interlace_handling(handle->reader);
  png_read_update_info(handle->reader,handle->info);
  if(png_get_rowbytes(handle->reader,handle->info)!=shape.width) png_error(handle->reader,"Unexpected decoded row size");
  png_read_image(handle->reader,rows.data());
  png_read_end(handle->reader,nullptr);
  return result;
}
bool same_shape(const ImageInfo& a, const ImageInfo& b) {
  return a.width == b.width && a.height == b.height && a.bit_depth == b.bit_depth && a.channels == b.channels;
}
} // namespace

std::int64_t parse_timestamp(std::string_view value, std::string_view context) {
  std::int64_t parsed{};
  if (value.empty() || value.find_first_not_of("0123456789") != value.npos)
    throw Error(2, std::string(context) + ": timestamp must be nonnegative integer nanoseconds");
  const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size())
    throw Error(2, std::string(context) + ": timestamp exceeds int64 range");
  return parsed;
}
double relative_seconds(std::int64_t timestamp, std::int64_t origin) {
  if (timestamp < 0 || origin < 0) throw Error(2, "Negative source timestamp");
  return static_cast<double>(timestamp - origin) * 1e-9;
}
ImageInfo probe_png(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw Error(5, "Cannot read PNG: " + path.string());
  std::array<unsigned char, 33> bytes{};
  input.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
  constexpr std::array<unsigned char, 8> signature{137,80,78,71,13,10,26,10};
  if (input.gcount() != 33 || !std::equal(signature.begin(), signature.end(), bytes.begin()) ||
      big_endian(bytes.data()+8) != 13 || std::string_view(reinterpret_cast<char*>(bytes.data()+12), 4) != "IHDR")
    throw Error(2, "Invalid PNG header: " + path.string());
  if (crc32(0, bytes.data()+12, 17) != big_endian(bytes.data()+29))
    throw Error(2, "Invalid PNG header CRC: " + path.string());
  ImageInfo info{big_endian(bytes.data()+16), big_endian(bytes.data()+20), 0, bytes[24]};
  if (!info.width || !info.height || info.width > 32768 || info.height > 32768)
    throw Error(2, "PNG dimensions outside supported range 1..32768: " + path.string());
  if (bytes[25] == 0) info.channels = 1;
  else if (bytes[25] == 2) info.channels = 3;
  else throw Error(8, "PNG must be grayscale or RGB (no palette/alpha yet): " + path.string());
  if (info.bit_depth != 8 && !(info.bit_depth == 16 && info.channels == 1))
    throw Error(8, "Only PNG mono8/mono16/RGB8 supported: " + path.string());
  if (bytes[26] || bytes[27] || bytes[28] > 1)
    throw Error(2, "Unsupported PNG compression/filter/interlace: " + path.string());
  return info;
}
GrayImage decode_png(const fs::path& path, std::size_t max_pixels) {
  const auto info = probe_png(path);
  const auto count = std::size_t(info.width) * info.height;
  if (count > max_pixels) throw Error(2, "Image exceeds configured working-set budget: " + path.string());
  if (info.bit_depth == 16) return decode_mono16(path,info);
  PngHandle handle;
  if (!png_image_begin_read_from_file(&handle.image, path.c_str()))
    throw Error(2, "PNG decode failed: " + path.string() + ": " + handle.image.message);
  if (handle.image.width != info.width || handle.image.height != info.height)
    throw Error(2, "PNG changed while reading: " + path.string());
  handle.image.format = PNG_FORMAT_GRAY;
  GrayImage image{info, std::vector<std::uint8_t>(count)};
  if (!png_image_finish_read(&handle.image, nullptr, image.pixels.data(), 0, nullptr))
    throw Error(2, "PNG decode failed: " + path.string() + ": " + handle.image.message);
  if (handle.image.warning_or_error)
    throw Error(2, "PNG warning rejected: " + path.string() + ": " + handle.image.message);
  return image;
}

Json read_imu(const fs::path& path, const std::function<void(const ImuSample&)>& consume) {
  std::ifstream input(path);
  if (!input) throw Error(5, "Cannot read IMU CSV: " + path.string());
  std::string line;
  if (!std::getline(input, line)) throw Error(2, "Missing IMU CSV header: " + path.string());
  const auto header = split(line);
  const std::array<std::string,7> required{"timestamp_ns","wx","wy","wz","ax","ay","az"};
  if (header.size() != required.size()) throw Error(2, "IMU CSV must have exactly seven named columns");
  std::array<std::size_t,7> columns{};
  for (std::size_t i=0; i<required.size(); ++i) {
    if (std::count(header.begin(), header.end(), required[i]) != 1)
      throw Error(2, "IMU CSV missing or duplicate column: " + required[i]);
    columns[i] = static_cast<std::size_t>(std::find(header.begin(),header.end(),required[i])-header.begin());
  }
  std::size_t number=1, count=0;
  std::int64_t first=0, previous=0, min_dt=std::numeric_limits<std::int64_t>::max(), max_dt=0;
  std::array<double,6> mean{}, m2{}, minimum{}, maximum{};
  while (std::getline(input,line)) {
    ++number;
    if (trim(line).empty()) continue;
    const auto context = path.string()+":"+std::to_string(number);
    if (line.size()>4096) throw Error(2, context+": CSV line too long");
    const auto cells = split(line);
    if (cells.size()!=7) throw Error(2, context+": expected seven fields");
    ImuSample sample;
    sample.timestamp_ns=parse_timestamp(cells[columns[0]],context);
    if (count && sample.timestamp_ns<=previous) throw Error(2, context+": IMU timestamps must be strictly increasing");
    for (std::size_t i=0;i<6;++i) {
      const auto& value=cells[columns[i+1]];
      const auto converted=std::from_chars(value.data(),value.data()+value.size(),sample.values[i]);
      if (value.empty() || converted.ec!=std::errc{} || converted.ptr!=value.data()+value.size() || !std::isfinite(sample.values[i]))
        throw Error(2, context+": invalid finite numeric value in "+required[i+1]);
      if (count==0) minimum[i]=maximum[i]=sample.values[i];
      minimum[i]=std::min(minimum[i],sample.values[i]);
      maximum[i]=std::max(maximum[i],sample.values[i]);
      const double delta=sample.values[i]-mean[i];
      mean[i]+=delta/static_cast<double>(count+1);
      m2[i]+=delta*(sample.values[i]-mean[i]);
      if (!std::isfinite(mean[i]) || !std::isfinite(m2[i])) throw Error(2, context+": numeric range overflow");
    }
    if (!count) first=sample.timestamp_ns;
    else { const auto dt=sample.timestamp_ns-previous; min_dt=std::min(min_dt,dt); max_dt=std::max(max_dt,dt); }
    previous=sample.timestamp_ns;
    ++count;
    if (consume) consume(sample);
  }
  if (input.bad()) throw Error(5,"I/O failure: "+path.string());
  if (count<2) throw Error(3,"Need at least two IMU samples for timing inspection");
  std::array<double,6> stddev{};
  for (std::size_t i=0;i<6;++i) stddev[i]=std::sqrt(m2[i]/static_cast<double>(count-1));
  const auto duration=relative_seconds(previous,first);
  return {{"status","inspected"},{"samples",count},{"first_timestamp_ns",first},{"last_timestamp_ns",previous},
          {"duration_s",duration},{"mean_rate_hz",static_cast<double>(count-1)/duration},
          {"min_interval_ns",min_dt},{"max_interval_ns",max_dt},{"mean",mean},{"stddev",stddev},
          {"min",minimum},{"max",maximum},{"saturation_assessment","unavailable: sensor range not provided"}};
}
Dataset NativeDirectoryReader::index(const std::vector<std::string>& selected, bool require_imu) const {
  if (!fs::is_directory(root_)) throw Error(2,"Dataset directory does not exist: "+root_.string());
  Dataset result{fs::absolute(root_),{},{{"status", "not_requested"}}};
  std::map<std::int64_t,std::string> discovered;
  for (const auto& entry:fs::directory_iterator(root_)) {
    const auto name=entry.path().filename().string();
    if (!entry.is_directory() || !name.starts_with("cam")) continue;
    const auto suffix=std::string_view(name).substr(3);
    if (suffix.empty() || suffix.find_first_not_of("0123456789")!=suffix.npos) continue;
    const auto index=parse_timestamp(suffix,name);
    if (suffix.size()>1 && suffix[0]=='0') throw Error(2,"Noncanonical camera directory: "+name);
    discovered.emplace(index,name);
  }
  std::vector<std::string> ids=selected;
  if (ids.empty()) for (const auto& [index,name]:discovered) { (void)index; ids.push_back(name); }
  if (ids.empty()) throw Error(3,"No camN directories found");
  std::set<std::string> seen;
  for (const auto& id:ids) {
    if (!seen.insert(id).second) throw Error(2,"Repeated camera selection: "+id);
    if (std::none_of(discovered.begin(),discovered.end(),[&](const auto& item){return item.second==id;}))
      throw Error(2,"Unknown camera selection: "+id);
    CameraStream stream{id,{}};
    for (const auto& entry:fs::directory_iterator(root_/id)) {
      if (entry.path().extension()!=".png") continue;
      if (!entry.is_regular_file()) throw Error(2,"PNG is not a regular file: "+entry.path().string());
      const auto time=parse_timestamp(entry.path().stem().string(),entry.path().string());
      stream.frames.push_back({id,time,entry.path(),probe_png(entry.path())});
    }
    std::ranges::sort(stream.frames,{},&FrameRef::timestamp_ns);
    if (stream.frames.empty()) throw Error(3,"No PNG images in "+id);
    for (std::size_t i=0;i<stream.frames.size();++i) {
      if (i && stream.frames[i].timestamp_ns==stream.frames[i-1].timestamp_ns)
        throw Error(2,"Duplicate image timestamp: "+stream.frames[i].path.string());
      if (!same_shape(stream.frames[i].image,stream.frames[0].image))
        throw Error(2,"Image dimensions/encoding changed within "+id);
    }
    result.cameras.push_back(std::move(stream));
  }
  if (require_imu) {
    result.imu=read_imu(root_/"imu0"/"imu0.csv");
    const auto begin=result.imu.at("first_timestamp_ns").get<std::int64_t>();
    const auto end=result.imu.at("last_timestamp_ns").get<std::int64_t>();
    for (const auto& stream:result.cameras)
      if (stream.frames.back().timestamp_ns<begin || stream.frames.front().timestamp_ns>end)
        throw Error(3,"No raw timestamp overlap between "+stream.id+" and imu0 (no offset correction applied)");
  }
  return result;
}
GrayImage NativeDirectoryReader::decode(const FrameRef& frame,std::size_t max_pixels) const {
  auto image=decode_png(frame.path,max_pixels);
  if (!same_shape(frame.image,image.info)) throw Error(2,"Image changed after indexing: "+frame.path.string());
  return image;
}
Json dataset_summary(const Dataset& dataset) {
  Json result{{"dataset",dataset.root.string()},{"cameras",Json::array()},{"imu",dataset.imu}};
  for (const auto& camera:dataset.cameras) {
    const auto& first=camera.frames.front();
    const auto& last=camera.frames.back();
    Json stream{{"id",camera.id},{"frames",camera.frames.size()},
                {"resolution",{first.image.width,first.image.height}},{"bit_depth",first.image.bit_depth},
                {"channels",first.image.channels},{"first_timestamp_ns",first.timestamp_ns},
                {"last_timestamp_ns",last.timestamp_ns},{"duration_s",relative_seconds(last.timestamp_ns,first.timestamp_ns)}};
    if (camera.frames.size()>1) {
      std::int64_t lo=std::numeric_limits<std::int64_t>::max(),hi=0;
      for(std::size_t i=1;i<camera.frames.size();++i) {
        auto dt=camera.frames[i].timestamp_ns-camera.frames[i-1].timestamp_ns;
        lo=std::min(lo,dt); hi=std::max(hi,dt);
      }
      stream["min_interval_ns"]=lo; stream["max_interval_ns"]=hi;
      stream["mean_rate_hz"]=static_cast<double>(camera.frames.size()-1)/relative_seconds(last.timestamp_ns,first.timestamp_ns);
    }
    result["cameras"].push_back(std::move(stream));
  }
  return result;
}
} // namespace calib
