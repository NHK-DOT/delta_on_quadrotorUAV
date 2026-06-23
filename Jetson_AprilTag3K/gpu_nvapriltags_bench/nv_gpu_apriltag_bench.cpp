#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "nvAprilTags.h"

namespace {

struct Args {
  int sensor_mode = 0;
  int sensor_w = 3264;
  int sensor_h = 2464;
  int sensor_fps = 21;
  int tnr_mode = -1;
  float tnr_strength = -1.0f;
  float exposure_compensation = 0.0f;
  int out_w = 800;
  int out_h = 604;
  int seconds = 20;
  int warmup = 20;
  bool gui = false;
  bool draw_axes = false;
  bool pose = false;
  std::string preprocess = "raw";
  float hfov_deg = 0.0f;
  float fx = 0.0f;
  float fy = 0.0f;
  float cx = 0.0f;
  float cy = 0.0f;
  float tag_size = 0.0305f;
  float gui_hold_ms = 120.0f;
  float output_hold_ms = 80.0f;
  std::string exposuretimerange;
  std::string gainrange;
  std::string ispdigitalgainrange;
  std::string calib_json;
  std::string output_json;
};

double now_ms() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double, std::milli>(clock::now().time_since_epoch()).count();
}

bool parse_int(const char* s, int* out) {
  char* end = nullptr;
  long v = std::strtol(s, &end, 10);
  if (!end || *end != '\0') return false;
  *out = static_cast<int>(v);
  return true;
}

bool parse_float(const char* s, float* out) {
  char* end = nullptr;
  float v = std::strtof(s, &end);
  if (!end || *end != '\0') return false;
  *out = v;
  return true;
}

void usage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << " [options]\n"
      << "  --mode <0|1|2|3|4>      nvarguscamerasrc sensor-mode\n"
      << "  --sensor WxH             sensor capture size, e.g. 3264x2464\n"
      << "  --sensor-fps N           sensor framerate\n"
      << "  --exposure-compensation N  nvarguscamerasrc exposure compensation, -2..2\n"
      << "  --exposuretimerange \"L H\" nvarguscamerasrc exposure time range ns\n"
      << "  --gainrange \"L H\"         nvarguscamerasrc analog gain range\n"
      << "  --ispdigitalgainrange \"L H\" nvarguscamerasrc ISP digital gain range\n"
      << "  --tnr-mode N             temporal noise reduction: 0 off, 1 fast, 2 high quality\n"
      << "  --tnr-strength N         temporal noise reduction strength\n"
      << "  --out WxH                nvvidconv output size, e.g. 800x604\n"
      << "  --seconds N              measured run duration\n"
      << "  --warmup N               warmup frames\n"
      << "  --gui                    show local OpenCV GUI until q/Esc or seconds ends\n"
      << "  --draw-axes              draw 3D tag orientation axes in GUI\n"
      << "  --preprocess MODE        detector input preprocess:\n"
      << "                            raw|equalize|clahe|gamma06|adaptive|\n"
      << "                            color_gamma06|color_gamma045|gain|y_equalize|y_clahe|\n"
      << "                            gray_blur_gamma045|gray_blur_gamma05|gray_blur_gamma06|gray_blur_gamma07|\n"
      << "                            gray_median_gamma06|gray_sharp_gamma06\n"
      << "  --hfov-deg N             approximate pinhole intrinsics from horizontal FOV\n"
      << "  --calib-json PATH        load and scale camera_matrix from calibration JSON\n"
      << "  --pose fx,fy,cx,cy,tag   enable pose output with intrinsics and tag size\n"
      << "  --gui-hold-ms N          draw last detected tag for N ms in GUI only\n"
      << "  --output-hold-ms N       reuse last JSON detection for N ms after a miss\n"
      << "  --output-json PATH       write latest detections for arm-side Python\n";
}

std::vector<double> extract_numbers_for_key(const std::string& text, const std::string& key) {
  std::vector<double> values;
  size_t key_pos = text.find("\"" + key + "\"");
  if (key_pos == std::string::npos) key_pos = text.find(key);
  if (key_pos == std::string::npos) return values;
  size_t start = text.find('[', key_pos);
  if (start == std::string::npos) return values;
  int depth = 0;
  size_t end = start;
  for (; end < text.size(); ++end) {
    if (text[end] == '[') depth++;
    if (text[end] == ']') {
      depth--;
      if (depth == 0) {
        end++;
        break;
      }
    }
  }
  std::string block = text.substr(start, end - start);
  const char* p = block.c_str();
  while (*p) {
    char* next = nullptr;
    double v = std::strtod(p, &next);
    if (next != p) {
      values.push_back(v);
      p = next;
    } else {
      ++p;
    }
  }
  return values;
}

int extract_int_after_key(const std::string& text, const std::string& key) {
  size_t key_pos = text.find("\"" + key + "\"");
  if (key_pos == std::string::npos) key_pos = text.find(key);
  if (key_pos == std::string::npos) return 0;
  size_t colon = text.find(':', key_pos);
  if (colon == std::string::npos) return 0;
  const char* p = text.c_str() + colon + 1;
  char* next = nullptr;
  long v = std::strtol(p, &next, 10);
  if (next == p) return 0;
  return static_cast<int>(v);
}

bool load_scaled_calibration(const std::string& path, int out_w, int out_h, Args* args) {
  std::ifstream in(path.c_str());
  if (!in) return false;
  std::stringstream buffer;
  buffer << in.rdbuf();
  std::string text = buffer.str();
  std::vector<double> k = extract_numbers_for_key(text, "camera_matrix");
  if (k.size() < 9) return false;
  int src_w = extract_int_after_key(text, "width");
  int src_h = extract_int_after_key(text, "height");
  double sx = src_w > 0 ? static_cast<double>(out_w) / src_w : 1.0;
  double sy = src_h > 0 ? static_cast<double>(out_h) / src_h : 1.0;
  args->fx = static_cast<float>(k[0] * sx);
  args->fy = static_cast<float>(k[4] * sy);
  args->cx = static_cast<float>(k[2] * sx);
  args->cy = static_cast<float>(k[5] * sy);
  args->pose = true;
  return true;
}

bool parse_size(const char* s, int* w, int* h) {
  std::string value(s);
  auto pos = value.find('x');
  if (pos == std::string::npos) return false;
  int tw = 0;
  int th = 0;
  if (!parse_int(value.substr(0, pos).c_str(), &tw)) return false;
  if (!parse_int(value.substr(pos + 1).c_str(), &th)) return false;
  *w = tw;
  *h = th;
  return true;
}

bool parse_pose(const char* s, Args* args) {
  std::stringstream ss(s);
  std::string item;
  std::vector<float> vals;
  while (std::getline(ss, item, ',')) {
    float v = 0.0f;
    if (!parse_float(item.c_str(), &v)) return false;
    vals.push_back(v);
  }
  if (vals.size() != 5) return false;
  args->pose = true;
  args->fx = vals[0];
  args->fy = vals[1];
  args->cx = vals[2];
  args->cy = vals[3];
  args->tag_size = vals[4];
  return true;
}

bool parse_args(int argc, char** argv, Args* args) {
  for (int i = 1; i < argc; ++i) {
    std::string key(argv[i]);
    auto need_value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << name << "\n";
        return nullptr;
      }
      return argv[++i];
    };
    if (key == "--mode") {
      const char* v = need_value("--mode");
      if (!v || !parse_int(v, &args->sensor_mode)) return false;
    } else if (key == "--sensor") {
      const char* v = need_value("--sensor");
      if (!v || !parse_size(v, &args->sensor_w, &args->sensor_h)) return false;
    } else if (key == "--sensor-fps") {
      const char* v = need_value("--sensor-fps");
      if (!v || !parse_int(v, &args->sensor_fps)) return false;
    } else if (key == "--exposure-compensation") {
      const char* v = need_value("--exposure-compensation");
      if (!v || !parse_float(v, &args->exposure_compensation)) return false;
    } else if (key == "--exposuretimerange") {
      const char* v = need_value("--exposuretimerange");
      if (!v) return false;
      args->exposuretimerange = v;
    } else if (key == "--gainrange") {
      const char* v = need_value("--gainrange");
      if (!v) return false;
      args->gainrange = v;
    } else if (key == "--ispdigitalgainrange") {
      const char* v = need_value("--ispdigitalgainrange");
      if (!v) return false;
      args->ispdigitalgainrange = v;
    } else if (key == "--tnr-mode") {
      const char* v = need_value("--tnr-mode");
      if (!v || !parse_int(v, &args->tnr_mode)) return false;
    } else if (key == "--tnr-strength") {
      const char* v = need_value("--tnr-strength");
      if (!v || !parse_float(v, &args->tnr_strength)) return false;
    } else if (key == "--out") {
      const char* v = need_value("--out");
      if (!v || !parse_size(v, &args->out_w, &args->out_h)) return false;
    } else if (key == "--seconds") {
      const char* v = need_value("--seconds");
      if (!v || !parse_int(v, &args->seconds)) return false;
    } else if (key == "--warmup") {
      const char* v = need_value("--warmup");
      if (!v || !parse_int(v, &args->warmup)) return false;
    } else if (key == "--gui") {
      args->gui = true;
    } else if (key == "--draw-axes") {
      args->draw_axes = true;
    } else if (key == "--preprocess") {
      const char* v = need_value("--preprocess");
      if (!v) return false;
      args->preprocess = v;
      if (args->preprocess != "raw" &&
          args->preprocess != "equalize" &&
          args->preprocess != "clahe" &&
          args->preprocess != "gamma06" &&
          args->preprocess != "adaptive" &&
          args->preprocess != "color_gamma06" &&
          args->preprocess != "color_gamma045" &&
          args->preprocess != "gain" &&
          args->preprocess != "y_equalize" &&
          args->preprocess != "y_clahe" &&
          args->preprocess != "gray_blur_gamma045" &&
          args->preprocess != "gray_blur_gamma05" &&
          args->preprocess != "gray_blur_gamma06" &&
          args->preprocess != "gray_blur_gamma07" &&
          args->preprocess != "gray_median_gamma06" &&
          args->preprocess != "gray_sharp_gamma06") {
        std::cerr << "unknown preprocess mode: " << args->preprocess << "\n";
        return false;
      }
    } else if (key == "--hfov-deg") {
      const char* v = need_value("--hfov-deg");
      if (!v || !parse_float(v, &args->hfov_deg)) return false;
    } else if (key == "--calib-json") {
      const char* v = need_value("--calib-json");
      if (!v) return false;
      args->calib_json = v;
    } else if (key == "--pose") {
      const char* v = need_value("--pose");
      if (!v || !parse_pose(v, args)) return false;
    } else if (key == "--gui-hold-ms") {
      const char* v = need_value("--gui-hold-ms");
      if (!v || !parse_float(v, &args->gui_hold_ms)) return false;
    } else if (key == "--output-hold-ms") {
      const char* v = need_value("--output-hold-ms");
      if (!v || !parse_float(v, &args->output_hold_ms)) return false;
    } else if (key == "--output-json") {
      const char* v = need_value("--output-json");
      if (!v) return false;
      args->output_json = v;
    } else if (key == "--help" || key == "-h") {
      usage(argv[0]);
      std::exit(0);
    } else {
      std::cerr << "unknown option: " << key << "\n";
      return false;
    }
  }
  return true;
}

std::string make_pipeline(const Args& args) {
  std::ostringstream p;
  p << "nvarguscamerasrc sensor-id=0 sensor-mode=" << args.sensor_mode << " "
    << "exposurecompensation=" << args.exposure_compensation << " ";
  if (!args.exposuretimerange.empty()) {
    p << "exposuretimerange=\"" << args.exposuretimerange << "\" ";
  }
  if (!args.gainrange.empty()) {
    p << "gainrange=\"" << args.gainrange << "\" ";
  }
  if (!args.ispdigitalgainrange.empty()) {
    p << "ispdigitalgainrange=\"" << args.ispdigitalgainrange << "\" ";
  }
  if (args.tnr_mode >= 0) {
    p << "tnr-mode=" << args.tnr_mode << " ";
  }
  if (args.tnr_strength >= 0.0f) {
    p << "tnr-strength=" << args.tnr_strength << " ";
  }
  p << "! "
    << "video/x-raw(memory:NVMM),width=" << args.sensor_w
    << ",height=" << args.sensor_h
    << ",framerate=" << args.sensor_fps << "/1 ! "
    << "nvvidconv flip-method=0 ! "
    << "video/x-raw,width=" << args.out_w
    << ",height=" << args.out_h
    << ",format=I420 ! "
    << "videoconvert ! video/x-raw,format=BGR ! "
    << "appsink drop=true max-buffers=1 sync=false";
  return p.str();
}

double avg(const std::vector<double>& v) {
  if (v.empty()) return 0.0;
  return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

double pct(const std::vector<double>& v, double q) {
  if (v.empty()) return 0.0;
  std::vector<double> sorted(v);
  std::sort(sorted.begin(), sorted.end());
  size_t idx = static_cast<size_t>((sorted.size() - 1) * q);
  return sorted[idx];
}

cv::Point2f project_point(const Args& args, float x, float y, float z) {
  if (z <= 1.0e-6f) return cv::Point2f(-1.0f, -1.0f);
  return cv::Point2f(args.fx * x / z + args.cx, args.fy * y / z + args.cy);
}

cv::Point2f tag_center(const nvAprilTagsID_t& tag) {
  cv::Point2f c(0.0f, 0.0f);
  for (int i = 0; i < 4; ++i) {
    c.x += tag.corners[i].x;
    c.y += tag.corners[i].y;
  }
  c.x *= 0.25f;
  c.y *= 0.25f;
  return c;
}

double tag_area(const nvAprilTagsID_t& tag) {
  double area = 0.0;
  for (int i = 0; i < 4; ++i) {
    int j = (i + 1) % 4;
    area += static_cast<double>(tag.corners[i].x) * tag.corners[j].y -
            static_cast<double>(tag.corners[j].x) * tag.corners[i].y;
  }
  return std::fabs(area) * 0.5;
}

bool better_tag_candidate(const nvAprilTagsID_t& candidate, const nvAprilTagsID_t& current) {
  if (candidate.hamming_error != current.hamming_error) {
    return candidate.hamming_error < current.hamming_error;
  }
  return tag_area(candidate) > tag_area(current);
}

uint32_t filter_duplicate_ids(
    const std::vector<nvAprilTagsID_t>& input,
    uint32_t input_count,
    uint32_t max_tags,
    std::vector<nvAprilTagsID_t>* output) {
  output->clear();
  uint32_t limit = std::min<uint32_t>(input_count, std::min<uint32_t>(max_tags, input.size()));
  for (uint32_t i = 0; i < limit; ++i) {
    const nvAprilTagsID_t& tag = input[i];
    bool replaced = false;
    for (auto& existing : *output) {
      if (existing.id == tag.id) {
        if (better_tag_candidate(tag, existing)) {
          existing = tag;
        }
        replaced = true;
        break;
      }
    }
    if (!replaced) {
      output->push_back(tag);
    }
  }
  return static_cast<uint32_t>(output->size());
}

double unix_time_s() {
  using clock = std::chrono::system_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

std::string pixel_mode_for_preprocess(const std::string& preprocess) {
  if (preprocess == "raw") return "BGR_to_BGRA_cuda";
  if (preprocess == "equalize") return "BGR_gray_equalizeHist_to_BGRA_cuda";
  if (preprocess == "clahe") return "BGR_gray_CLAHE_to_BGRA_cuda";
  if (preprocess == "gamma06") return "BGR_gray_gamma0.60_to_BGRA_cuda";
  if (preprocess == "adaptive") return "BGR_gray_adaptiveThreshold_to_BGRA_cuda";
  if (preprocess == "color_gamma06") return "BGR_color_gamma0.60_to_BGRA_cuda";
  if (preprocess == "color_gamma045") return "BGR_color_gamma0.45_to_BGRA_cuda";
  if (preprocess == "gain") return "BGR_alpha1.65_beta18_to_BGRA_cuda";
  if (preprocess == "y_equalize") return "BGR_YCrCb_Y_equalizeHist_to_BGRA_cuda";
  if (preprocess == "y_clahe") return "BGR_YCrCb_Y_CLAHE_to_BGRA_cuda";
  if (preprocess == "gray_blur_gamma045") return "BGR_gray_blur_gamma0.45_to_BGRA_cuda";
  if (preprocess == "gray_blur_gamma05") return "BGR_gray_blur_gamma0.50_to_BGRA_cuda";
  if (preprocess == "gray_blur_gamma06") return "BGR_gray_blur_gamma0.60_to_BGRA_cuda";
  if (preprocess == "gray_blur_gamma07") return "BGR_gray_blur_gamma0.70_to_BGRA_cuda";
  if (preprocess == "gray_median_gamma06") return "BGR_gray_median_gamma0.60_to_BGRA_cuda";
  if (preprocess == "gray_sharp_gamma06") return "BGR_gray_sharpen_gamma0.60_to_BGRA_cuda";
  return "BGR_to_BGRA_cuda";
}

bool write_snapshot_json(
    const std::string& path,
    const Args& args,
    const std::vector<nvAprilTagsID_t>& tags,
    uint32_t num_tags,
    double fps_ema,
    double read_ms,
    double preprocess_ms,
    double copy_ms,
    double detect_ms,
    bool detections_held,
    double held_ms,
    double source_timestamp_unix) {
  if (path.empty()) return true;
  std::string tmp = path + ".tmp";
  std::ofstream out(tmp.c_str());
  if (!out) return false;

  out << std::fixed << std::setprecision(6);
  out << "{\n";
  out << "  \"timestamp_unix\": " << unix_time_s() << ",\n";
  out << "  \"camera\": {\n";
  out << "    \"source\": \"csi_nvargus_nvapriltags_gpu\",\n";
  out << "    \"sensor_id\": 0,\n";
  out << "    \"sensor_mode\": " << args.sensor_mode << ",\n";
  out << "    \"sensor_width\": " << args.sensor_w << ",\n";
  out << "    \"sensor_height\": " << args.sensor_h << ",\n";
  out << "    \"sensor_fps_request\": " << args.sensor_fps << ",\n";
  out << "    \"processing_width\": " << args.out_w << ",\n";
  out << "    \"processing_height\": " << args.out_h << ",\n";
  out << "    \"pixel_mode\": \"" << pixel_mode_for_preprocess(args.preprocess) << "\",\n";
  out << "    \"detector_preprocess\": \"" << args.preprocess << "\"\n";
  out << "  },\n";
  out << "  \"tag_family\": \"tag36h11\",\n";
  out << "  \"tag_size_m\": " << args.tag_size << ",\n";
  out << "  \"estimation_mode\": \"nvidia_nvapriltags_gpu_pinhole_intrinsics\",\n";
  out << "  \"implementation\": \"nvapriltags_gpu_fullfov_downsample\",\n";
  out << "  \"calibration\": {\n";
  out << "    \"loaded\": " << (args.pose ? "true" : "false") << ",\n";
  out << "    \"file\": \"" << args.calib_json << "\",\n";
  out << "    \"fx\": " << args.fx << ",\n";
  out << "    \"fy\": " << args.fy << ",\n";
  out << "    \"cx\": " << args.cx << ",\n";
  out << "    \"cy\": " << args.cy << "\n";
  out << "  },\n";
  out << "  \"processing_frame\": {\"width\": " << args.out_w
      << ", \"height\": " << args.out_h << "},\n";
  out << "  \"timing\": {\n";
  out << "    \"display_fps\": " << fps_ema << ",\n";
  out << "    \"read_ms\": " << read_ms << ",\n";
  out << "    \"preprocess_ms\": " << preprocess_ms << ",\n";
  out << "    \"copy_ms\": " << copy_ms << ",\n";
  out << "    \"detector_ms\": " << detect_ms << "\n";
  out << "  },\n";
  out << "  \"detections\": [\n";
  uint32_t limit = std::min<uint32_t>(num_tags, tags.size());
  for (uint32_t i = 0; i < limit; ++i) {
    const auto& tag = tags[i];
    cv::Point2f center = tag_center(tag);
    out << "    {\n";
    out << "      \"id\": " << tag.id << ",\n";
    out << "      \"hamming\": " << static_cast<int>(tag.hamming_error) << ",\n";
    out << "      \"is_held\": " << (detections_held ? "true" : "false") << ",\n";
    out << "      \"held_ms\": " << held_ms << ",\n";
    out << "      \"source_timestamp_unix\": " << source_timestamp_unix << ",\n";
    out << "      \"center_px\": {\"x\": " << center.x << ", \"y\": " << center.y << "},\n";
    out << "      \"normalized_xy\": {\"x\": "
        << ((center.x - args.out_w * 0.5f) / (args.out_w * 0.5f))
        << ", \"y\": " << ((center.y - args.out_h * 0.5f) / (args.out_h * 0.5f))
        << "},\n";
    out << "      \"position_m\": {\"x\": " << tag.translation[0]
        << ", \"y\": " << tag.translation[1]
        << ", \"z\": " << tag.translation[2] << "},\n";
    out << "      \"orientation_matrix_column_major\": [";
    for (int j = 0; j < 9; ++j) {
      if (j) out << ", ";
      out << tag.orientation[j];
    }
    out << "],\n";
    out << "      \"corners_px\": [";
    for (int c = 0; c < 4; ++c) {
      if (c) out << ", ";
      out << "{\"x\": " << tag.corners[c].x << ", \"y\": " << tag.corners[c].y << "}";
    }
    out << "]\n";
    out << "    }" << (i + 1 < limit ? "," : "") << "\n";
  }
  out << "  ]\n";
  out << "}\n";
  out.close();
  return std::rename(tmp.c_str(), path.c_str()) == 0;
}

void preprocess_for_detector(
    const cv::Mat& frame,
    const Args& args,
    const cv::Ptr<cv::CLAHE>& clahe,
    const cv::Mat& gamma06_lut,
    const cv::Mat& gamma045_lut,
    const cv::Mat& gamma05_lut,
    const cv::Mat& gamma07_lut,
    cv::Mat* gray,
    cv::Mat* processed_gray,
    cv::Mat* scratch_gray,
    cv::Mat* processed_bgr,
    cv::Mat* ycrcb,
    cv::Mat* upload_frame) {
  if (args.preprocess == "raw") {
    cv::cvtColor(frame, *upload_frame, cv::COLOR_BGR2BGRA);
    return;
  }

  if (args.preprocess == "color_gamma06") {
    cv::LUT(frame, gamma06_lut, *processed_bgr);
    cv::cvtColor(*processed_bgr, *upload_frame, cv::COLOR_BGR2BGRA);
    return;
  }
  if (args.preprocess == "color_gamma045") {
    cv::LUT(frame, gamma045_lut, *processed_bgr);
    cv::cvtColor(*processed_bgr, *upload_frame, cv::COLOR_BGR2BGRA);
    return;
  }
  if (args.preprocess == "gain") {
    frame.convertTo(*processed_bgr, -1, 1.65, 18.0);
    cv::cvtColor(*processed_bgr, *upload_frame, cv::COLOR_BGR2BGRA);
    return;
  }
  if (args.preprocess == "y_equalize" || args.preprocess == "y_clahe") {
    cv::cvtColor(frame, *ycrcb, cv::COLOR_BGR2YCrCb);
    std::vector<cv::Mat> channels;
    cv::split(*ycrcb, channels);
    if (args.preprocess == "y_equalize") {
      cv::equalizeHist(channels[0], channels[0]);
    } else {
      clahe->apply(channels[0], channels[0]);
    }
    cv::merge(channels, *ycrcb);
    cv::cvtColor(*ycrcb, *processed_bgr, cv::COLOR_YCrCb2BGR);
    cv::cvtColor(*processed_bgr, *upload_frame, cv::COLOR_BGR2BGRA);
    return;
  }

  cv::cvtColor(frame, *gray, cv::COLOR_BGR2GRAY);
  if (args.preprocess == "equalize") {
    cv::equalizeHist(*gray, *processed_gray);
  } else if (args.preprocess == "clahe") {
    clahe->apply(*gray, *processed_gray);
  } else if (args.preprocess == "gamma06") {
    cv::LUT(*gray, gamma06_lut, *processed_gray);
  } else if (args.preprocess == "gray_blur_gamma045") {
    cv::GaussianBlur(*gray, *processed_gray, cv::Size(3, 3), 0.0);
    cv::LUT(*processed_gray, gamma045_lut, *processed_gray);
  } else if (args.preprocess == "gray_blur_gamma05") {
    cv::GaussianBlur(*gray, *processed_gray, cv::Size(3, 3), 0.0);
    cv::LUT(*processed_gray, gamma05_lut, *processed_gray);
  } else if (args.preprocess == "gray_blur_gamma06") {
    cv::GaussianBlur(*gray, *processed_gray, cv::Size(3, 3), 0.0);
    cv::LUT(*processed_gray, gamma06_lut, *processed_gray);
  } else if (args.preprocess == "gray_blur_gamma07") {
    cv::GaussianBlur(*gray, *processed_gray, cv::Size(3, 3), 0.0);
    cv::LUT(*processed_gray, gamma07_lut, *processed_gray);
  } else if (args.preprocess == "gray_median_gamma06") {
    cv::medianBlur(*gray, *processed_gray, 3);
    cv::LUT(*processed_gray, gamma06_lut, *processed_gray);
  } else if (args.preprocess == "gray_sharp_gamma06") {
    cv::GaussianBlur(*gray, *scratch_gray, cv::Size(3, 3), 0.0);
    cv::addWeighted(*gray, 1.55, *scratch_gray, -0.55, 0.0, *processed_gray);
    cv::LUT(*processed_gray, gamma06_lut, *processed_gray);
  } else if (args.preprocess == "adaptive") {
    cv::adaptiveThreshold(
        *gray, *processed_gray, 255,
        cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 31, 5);
  } else {
    cv::equalizeHist(*gray, *processed_gray);
  }
  cv::cvtColor(*processed_gray, *upload_frame, cv::COLOR_GRAY2BGRA);
}

void draw_pose_axes(cv::Mat& frame, const Args& args, const nvAprilTagsID_t& tag) {
  if (!args.pose || tag.translation[2] <= 1.0e-6f) return;
  float axis = args.tag_size * 0.75f;
  auto transform = [&](float ax, float ay, float az) {
    // nvAprilTags returns column-major rotation.
    float x = tag.translation[0] + tag.orientation[0] * ax + tag.orientation[3] * ay + tag.orientation[6] * az;
    float y = tag.translation[1] + tag.orientation[1] * ax + tag.orientation[4] * ay + tag.orientation[7] * az;
    float z = tag.translation[2] + tag.orientation[2] * ax + tag.orientation[5] * ay + tag.orientation[8] * az;
    return project_point(args, x, y, z);
  };
  cv::Point2f origin = project_point(args, tag.translation[0], tag.translation[1], tag.translation[2]);
  if (origin.x < 0.0f || origin.y < 0.0f) return;
  cv::arrowedLine(frame, origin, transform(axis, 0.0f, 0.0f), cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.18);
  cv::arrowedLine(frame, origin, transform(0.0f, axis, 0.0f), cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.18);
  cv::arrowedLine(frame, origin, transform(0.0f, 0.0f, axis), cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0, 0.18);
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parse_args(argc, argv, &args)) {
    usage(argv[0]);
    return 2;
  }

  std::string pipeline = make_pipeline(args);
  std::cout << "pipeline=" << pipeline << "\n";

  if (!args.calib_json.empty()) {
    if (load_scaled_calibration(args.calib_json, args.out_w, args.out_h, &args)) {
      std::cout << "calibration=" << args.calib_json << " scaled fx=" << args.fx
                << " fy=" << args.fy << " cx=" << args.cx << " cy=" << args.cy << "\n";
    } else {
      std::cerr << "warning: failed to load calibration JSON: " << args.calib_json << "\n";
    }
  }
  if (!args.pose && args.hfov_deg > 0.0f) {
    float fx = (args.out_w * 0.5f) / std::tan((args.hfov_deg * static_cast<float>(CV_PI) / 180.0f) * 0.5f);
    args.fx = fx;
    args.fy = fx;
    args.cx = args.out_w * 0.5f;
    args.cy = args.out_h * 0.5f;
    args.pose = true;
    std::cout << "approx_intrinsics hfov_deg=" << args.hfov_deg << " fx=" << args.fx
              << " fy=" << args.fy << " cx=" << args.cx << " cy=" << args.cy << "\n";
  }

  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
  if (!cap.isOpened()) {
    std::cerr << "failed to open camera pipeline\n";
    return 1;
  }

  nvAprilTagsCameraIntrinsics_t cam{};
  nvAprilTagsCameraIntrinsics_t* cam_ptr = nullptr;
  if (args.pose) {
    cam.fx = args.fx;
    cam.fy = args.fy;
    cam.cx = args.cx;
    cam.cy = args.cy;
    cam_ptr = &cam;
  }

  nvAprilTagsHandle detector = nullptr;
  int rc = nvCreateAprilTagsDetector(
      &detector, static_cast<uint32_t>(args.out_w), static_cast<uint32_t>(args.out_h),
      NVAT_TAG36H11, cam_ptr, args.tag_size);
  if (rc != 0 || detector == nullptr) {
    std::cerr << "nvCreateAprilTagsDetector failed rc=" << rc << "\n";
    return 1;
  }

  uchar4* dev_ptr = nullptr;
  size_t pitch = 0;
  cudaError_t cu = cudaMallocPitch(
      reinterpret_cast<void**>(&dev_ptr), &pitch, args.out_w * sizeof(uchar4), args.out_h);
  if (cu != cudaSuccess) {
    std::cerr << "cudaMallocPitch failed: " << cudaGetErrorString(cu) << "\n";
    nvAprilTagsDestroy(detector);
    return 1;
  }

  nvAprilTagsImageInput_t input{};
  input.dev_ptr = dev_ptr;
  input.pitch = pitch;
  input.width = static_cast<uint16_t>(args.out_w);
  input.height = static_cast<uint16_t>(args.out_h);

  constexpr uint32_t kMaxTags = 64;
  std::vector<nvAprilTagsID_t> tags(kMaxTags);
  std::vector<nvAprilTagsID_t> filtered_tags;
  std::vector<nvAprilTagsID_t> json_hold_tags(kMaxTags);
  uint32_t num_tags = 0;
  uint32_t filtered_num_tags = 0;
  uint32_t json_hold_count = 0;
  double json_hold_until_ms = 0.0;
  double json_hold_source_unix = 0.0;

  cv::Mat frame;
  cv::Mat upload_frame;
  for (int i = 0; i < args.warmup; ++i) {
    if (!cap.read(frame) || frame.empty()) {
      std::cerr << "warmup read failed at frame " << i << "\n";
      cudaFree(dev_ptr);
      nvAprilTagsDestroy(detector);
      return 1;
    }
  }

  std::vector<double> read_ms;
  std::vector<double> preprocess_ms;
  std::vector<double> copy_ms;
  std::vector<double> detect_ms;
  std::vector<nvAprilTagsID_t> gui_hold_tags(kMaxTags);
  uint32_t gui_hold_count = 0;
  double gui_hold_until_ms = 0.0;
  if (args.gui) {
    cv::namedWindow("nvAprilTags GPU", cv::WINDOW_NORMAL);
    cv::resizeWindow("nvAprilTags GPU", args.out_w, args.out_h);
  }
  int frames = 0;
  int frames_with_tags = 0;
  double fps_ema = 0.0;
  double prev_frame_ms = now_ms();
  double start = now_ms();
  double deadline = args.seconds > 0 ? start + args.seconds * 1000.0 : 1.0e100;
  cv::Mat gray_frame;
  cv::Mat processed_gray;
  cv::Mat scratch_gray;
  cv::Mat processed_bgr;
  cv::Mat ycrcb;
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
  cv::Mat gamma06_lut(1, 256, CV_8UC1);
  cv::Mat gamma045_lut(1, 256, CV_8UC1);
  cv::Mat gamma05_lut(1, 256, CV_8UC1);
  cv::Mat gamma07_lut(1, 256, CV_8UC1);
  for (int i = 0; i < 256; ++i) {
    gamma06_lut.at<uchar>(0, i) = static_cast<uchar>(
        std::max(0.0, std::min(255.0, 255.0 * std::pow(i / 255.0, 0.60))));
    gamma045_lut.at<uchar>(0, i) = static_cast<uchar>(
        std::max(0.0, std::min(255.0, 255.0 * std::pow(i / 255.0, 0.45))));
    gamma05_lut.at<uchar>(0, i) = static_cast<uchar>(
        std::max(0.0, std::min(255.0, 255.0 * std::pow(i / 255.0, 0.50))));
    gamma07_lut.at<uchar>(0, i) = static_cast<uchar>(
        std::max(0.0, std::min(255.0, 255.0 * std::pow(i / 255.0, 0.70))));
  }

  while (now_ms() < deadline) {
    double t0 = now_ms();
    if (!cap.read(frame) || frame.empty()) {
      std::cerr << "read failed during measurement\n";
      break;
    }
    double t1 = now_ms();

    if (frame.cols != args.out_w || frame.rows != args.out_h || frame.type() != CV_8UC3) {
      std::cerr << "unexpected frame: " << frame.cols << "x" << frame.rows
                << " type=" << frame.type() << " expected CV_8UC3\n";
      break;
    }

    preprocess_for_detector(
        frame, args, clahe, gamma06_lut, gamma045_lut, gamma05_lut, gamma07_lut,
        &gray_frame, &processed_gray, &scratch_gray, &processed_bgr, &ycrcb, &upload_frame);
    double t1b = now_ms();

    cu = cudaMemcpy2D(
        dev_ptr, pitch, upload_frame.data, upload_frame.step, args.out_w * sizeof(uchar4), args.out_h,
        cudaMemcpyHostToDevice);
    if (cu != cudaSuccess) {
      std::cerr << "cudaMemcpy2D failed: " << cudaGetErrorString(cu) << "\n";
      break;
    }
    cu = cudaDeviceSynchronize();
    if (cu != cudaSuccess) {
      std::cerr << "cuda sync after copy failed: " << cudaGetErrorString(cu) << "\n";
      break;
    }
    double t2 = now_ms();

    num_tags = 0;
    rc = nvAprilTagsDetect(detector, &input, tags.data(), &num_tags, kMaxTags, nullptr);
    cu = cudaDeviceSynchronize();
    double t3 = now_ms();
    if (rc != 0 || cu != cudaSuccess) {
      std::cerr << "nvAprilTagsDetect failed rc=" << rc
                << " cuda=" << cudaGetErrorString(cu) << "\n";
      break;
    }
    filtered_num_tags = filter_duplicate_ids(tags, num_tags, kMaxTags, &filtered_tags);

    read_ms.push_back(t1 - t0);
    preprocess_ms.push_back(t1b - t1);
    copy_ms.push_back(t2 - t1b);
    detect_ms.push_back(t3 - t2);
    if (filtered_num_tags > 0) {
      frames_with_tags++;
      gui_hold_count = std::min<uint32_t>(filtered_num_tags, kMaxTags);
      for (uint32_t i = 0; i < gui_hold_count; ++i) {
        gui_hold_tags[i] = filtered_tags[i];
        json_hold_tags[i] = filtered_tags[i];
      }
      gui_hold_until_ms = t3 + std::max(0.0f, args.gui_hold_ms);
      json_hold_count = gui_hold_count;
      json_hold_until_ms = t3 + std::max(0.0f, args.output_hold_ms);
      json_hold_source_unix = unix_time_s();
    }
    frames++;

    double now = now_ms();
    double inst_fps = 1000.0 / std::max(1.0, now - prev_frame_ms);
    prev_frame_ms = now;
    fps_ema = fps_ema == 0.0 ? inst_fps : fps_ema * 0.9 + inst_fps * 0.1;

    const bool output_using_hold =
        (filtered_num_tags == 0 && json_hold_count > 0 && now_ms() < json_hold_until_ms);
    const std::vector<nvAprilTagsID_t>& output_tags = output_using_hold ? json_hold_tags : filtered_tags;
    const uint32_t output_count = output_using_hold ? json_hold_count : filtered_num_tags;
    const double held_ms = output_using_hold ? std::max(0.0, unix_time_s() - json_hold_source_unix) * 1000.0 : 0.0;
    const double source_timestamp = output_using_hold ? json_hold_source_unix : unix_time_s();
    if (!args.output_json.empty() &&
        !write_snapshot_json(
            args.output_json,
            args,
            output_tags,
            output_count,
            fps_ema,
            t1 - t0,
            t1b - t1,
            t2 - t1b,
            t3 - t2,
            output_using_hold,
            held_ms,
            source_timestamp)) {
      std::cerr << "warning: failed to write output json: " << args.output_json << "\n";
    }

    if (args.gui) {
      const bool using_hold = (filtered_num_tags == 0 && gui_hold_count > 0 && now_ms() < gui_hold_until_ms);
      const std::vector<nvAprilTagsID_t>& draw_tags = using_hold ? gui_hold_tags : filtered_tags;
      const uint32_t draw_count = using_hold ? gui_hold_count : std::min<uint32_t>(filtered_num_tags, kMaxTags);
      for (uint32_t i = 0; i < draw_count && i < kMaxTags; ++i) {
        std::vector<cv::Point> pts;
        pts.reserve(4);
        for (int c = 0; c < 4; ++c) {
          pts.emplace_back(
              static_cast<int>(std::lround(draw_tags[i].corners[c].x)),
              static_cast<int>(std::lround(draw_tags[i].corners[c].y)));
        }
        cv::Scalar line_color = using_hold ? cv::Scalar(0, 160, 255) : cv::Scalar(0, 255, 0);
        cv::polylines(frame, pts, true, line_color, 2, cv::LINE_AA);
        cv::Point2f center = tag_center(draw_tags[i]);
        cv::circle(frame, center, 4, cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_AA);
        if (args.draw_axes && !using_hold) {
          draw_pose_axes(frame, args, draw_tags[i]);
        }
        std::ostringstream tag_text;
        tag_text << (using_hold ? "hold id " : "id ") << draw_tags[i].id;
        if (args.pose && draw_tags[i].translation[2] > 1.0e-6f) {
          tag_text << std::fixed << std::setprecision(3)
                   << " X " << draw_tags[i].translation[0]
                   << " Y " << draw_tags[i].translation[1]
                   << " Z " << draw_tags[i].translation[2] << " m";
        }
        cv::putText(frame, tag_text.str(), pts[0] + cv::Point(4, -4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    using_hold ? cv::Scalar(0, 180, 255) : cv::Scalar(0, 255, 255),
                    2, cv::LINE_AA);
      }

      std::ostringstream overlay;
      overlay << std::fixed << std::setprecision(1)
              << "GPU AprilTag " << args.out_w << "x" << args.out_h
              << " fps=" << fps_ema
              << " tags=" << filtered_num_tags
              << " prep=" << args.preprocess
              << " detect=" << (t3 - t2) << "ms";
      if (num_tags == 0 && gui_hold_count > 0 && now_ms() < gui_hold_until_ms) {
        overlay << " hold=" << gui_hold_count;
      }
      if (args.pose) {
        overlay << " pose=on";
      }
      cv::rectangle(frame, cv::Rect(0, 0, std::min(args.out_w, 760), 34),
                    cv::Scalar(0, 0, 0), cv::FILLED);
      cv::putText(frame, overlay.str(), cv::Point(12, 24),
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
      cv::imshow("nvAprilTags GPU", frame);
      int key = cv::waitKey(1) & 0xff;
      if (key == 27 || key == 'q') break;
    }
  }

  double elapsed = (now_ms() - start) / 1000.0;
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "summary frames=" << frames
            << " elapsed_s=" << elapsed
            << " fps=" << (elapsed > 0.0 ? frames / elapsed : 0.0)
            << " frames_with_tags=" << frames_with_tags << "\n";
  std::cout << "read_ms avg=" << avg(read_ms) << " p50=" << pct(read_ms, 0.50)
            << " p90=" << pct(read_ms, 0.90) << "\n";
  std::cout << "preprocess=" << args.preprocess << "\n";
  std::cout << "preprocess_ms avg=" << avg(preprocess_ms) << " p50=" << pct(preprocess_ms, 0.50)
            << " p90=" << pct(preprocess_ms, 0.90) << "\n";
  std::cout << "copy_ms avg=" << avg(copy_ms) << " p50=" << pct(copy_ms, 0.50)
            << " p90=" << pct(copy_ms, 0.90) << "\n";
  std::cout << "detect_ms avg=" << avg(detect_ms) << " p50=" << pct(detect_ms, 0.50)
            << " p90=" << pct(detect_ms, 0.90) << "\n";
  if (frames > 0 && num_tags > 0) {
    std::cout << "last_num_tags=" << num_tags << "\n";
    for (uint32_t i = 0; i < num_tags && i < 8; ++i) {
      const auto& tag = tags[i];
      std::cout << "tag id=" << tag.id << " hamming=" << static_cast<int>(tag.hamming_error)
                << " corners=(" << tag.corners[0].x << "," << tag.corners[0].y << ")"
                << "(" << tag.corners[1].x << "," << tag.corners[1].y << ")"
                << "(" << tag.corners[2].x << "," << tag.corners[2].y << ")"
                << "(" << tag.corners[3].x << "," << tag.corners[3].y << ")";
      if (args.pose) {
        std::cout << " t=(" << tag.translation[0] << "," << tag.translation[1]
                  << "," << tag.translation[2] << ")";
      }
      std::cout << "\n";
    }
  }

  cudaFree(dev_ptr);
  nvAprilTagsDestroy(detector);
  return 0;
}
