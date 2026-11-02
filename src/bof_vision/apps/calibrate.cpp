// Adapted from TongjiSuperPower/sp_vision_25 calibration/calibrate_camera.cpp (MIT).
// Copyright (c) 2025 TongjiSuperPower. See third_party/licenses/TongjiSuperPower-MIT.txt.
// Retains circle-grid object points, calibrateCamera and per-view reprojection.
// Adds raw snapshot/video input, chessboard support and a directly loadable YAML.
#include "bof_vision/pose.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {
int integer(const std::string& value) {
  std::size_t used; const int result=std::stoi(value,&used);
  if(used!=value.size()) throw std::invalid_argument("invalid integer: "+value);
  return result;
}
double number(const std::string& value) {
  std::size_t used; const double result=std::stod(value,&used);
  if(used!=value.size() || !std::isfinite(result)) throw std::invalid_argument("invalid number: "+value);
  return result;
}
std::vector<cv::Point3f> centers_3d(cv::Size pattern,float spacing_m) {
  std::vector<cv::Point3f> points;
  for(int row=0;row<pattern.height;++row)
    for(int col=0;col<pattern.width;++col) points.push_back({col*spacing_m,row*spacing_m,0});
  return points;
}
}
int main(int argc,char** argv) {
  try {
    namespace fs=std::filesystem;
    std::string images,video_path,pattern="circles",glob="*_raw.png",output,review_dir,board_config;
    int cols=0,rows=0,step=15,min_views=10;
    double spacing_mm=0;
    bool fix_k3=false;
    std::vector<std::pair<std::string,std::string>> options;
    for(int i=1;i<argc;++i) {
      const std::string key=argv[i];
      if(key=="--help") {
        std::cout << "bof_calibrate (--images DIR [--glob '*_raw.png'] | --video PATH [--step 15])\n"
          " --pattern circles|chessboard --cols N --rows N --spacing-mm MM --output camera.yaml\n"
          " [--board-config PATH] [--review-dir NEW_DIR] [--min-views 10] [--fix-k3]\n"
          "For chessboards cols/rows count INNER corners; for circles count dot centres.\n"
          "Board config accepts Tongji pattern_cols/pattern_rows/center_distance_mm.\n";
        return 0;
      }
      if(key=="--fix-k3") { fix_k3=true; continue; }
      if(i+1>=argc) throw std::invalid_argument("missing value for "+key);
      const std::string value=argv[++i];
      if(key=="--board-config") board_config=value;
      else options.emplace_back(key,value);
    }
    if(!board_config.empty()) {
      const auto yaml=YAML::LoadFile(board_config);
      cols=yaml["pattern_cols"].as<int>(); rows=yaml["pattern_rows"].as<int>();
      spacing_mm=yaml["center_distance_mm"].as<double>();
      if(yaml["pattern_type"]) pattern=yaml["pattern_type"].as<std::string>();
    }
    for(const auto& [key,value]:options) {
      if(key=="--images") images=value;
      else if(key=="--video") video_path=value;
      else if(key=="--glob") glob=value;
      else if(key=="--pattern") pattern=value;
      else if(key=="--cols") cols=integer(value);
      else if(key=="--rows") rows=integer(value);
      else if(key=="--spacing-mm") spacing_mm=number(value);
      else if(key=="--step") step=integer(value);
      else if(key=="--min-views") min_views=integer(value);
      else if(key=="--output") output=value;
      else if(key=="--review-dir") review_dir=value;
      else throw std::invalid_argument("unknown option: "+key);
    }
    if(images.empty()==video_path.empty() || output.empty()) throw std::invalid_argument("choose images or video and supply --output");
    if(cols<2 || rows<2 || cols>100 || rows>100 || !std::isfinite(spacing_mm) || spacing_mm<=0 || step<1 || min_views<6)
      throw std::invalid_argument("specify board dimensions/spacing; step >= 1 and min-views >= 6");
    if(pattern!="circles" && pattern!="chessboard") throw std::invalid_argument("pattern must be circles or chessboard");
    if(fs::exists(output)) throw std::runtime_error("output already exists: "+output);
    if(!review_dir.empty()) {
      if(fs::exists(review_dir)) throw std::runtime_error("review directory already exists");
      fs::create_directories(review_dir);
    }
    const cv::Size board(cols,rows);
    const auto object=centers_3d(board,static_cast<float>(spacing_mm/1000));
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::string> sources;
    cv::Size image_size;
    unsigned examined=0;
    auto collect=[&](const cv::Mat& image,const std::string& name) {
      if(image.empty()) { std::cout << "unreadable " << name << '\n'; return; }
      if(image_size.empty()) image_size=image.size();
      if(image.size()!=image_size) throw std::runtime_error("mixed image resolutions: "+name);
      cv::Mat gray;
      if(image.channels()==1) gray=image; else cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);
      std::vector<cv::Point2f> corners;
      const bool found=pattern=="circles" ? cv::findCirclesGrid(gray,board,corners,cv::CALIB_CB_SYMMETRIC_GRID)
        : cv::findChessboardCornersSB(gray,board,corners,cv::CALIB_CB_NORMALIZE_IMAGE);
      std::cout << (found ? "accepted " : "not_found ") << name << '\n' << std::flush;
      if(!review_dir.empty()) {
        auto drawing=image.clone();
        cv::drawChessboardCorners(drawing,board,corners,found);
        if(!cv::imwrite((fs::path(review_dir)/(std::to_string(examined)+(found ? "_ok.png" : "_missing.png"))).string(),drawing))
          throw std::runtime_error("cannot write calibration review image");
      }
      ++examined;
      if(found) { image_points.push_back(corners); object_points.push_back(object); sources.push_back(name); }
    };
    if(!images.empty()) {
      std::vector<cv::String> files;
      cv::glob((fs::path(images)/glob).string(),files,false);
      for(const auto& file:files) collect(cv::imread(file),fs::absolute(file).string());
    } else {
      cv::VideoCapture video(video_path);
      if(!video.isOpened()) throw std::runtime_error("cannot open calibration video");
      cv::Mat frame; std::uint64_t index=0;
      while(video.read(frame)) {
        if(index%static_cast<unsigned>(step)==0) collect(frame,fs::absolute(video_path).string()+"#"+std::to_string(index));
        ++index;
      }
    }
    if(image_points.size()<static_cast<std::size_t>(min_views))
      throw std::runtime_error("only "+std::to_string(image_points.size())+" usable views; need "+std::to_string(min_views));
    cv::Mat matrix,distortion;
    std::vector<cv::Mat> rotations,translations;
    const double rms=cv::calibrateCamera(object_points,image_points,image_size,matrix,distortion,rotations,translations,
      fix_k3 ? cv::CALIB_FIX_K3 : 0,{cv::TermCriteria::COUNT+cv::TermCriteria::EPS,100,std::numeric_limits<double>::epsilon()});
    if(!std::isfinite(rms) || !cv::checkRange(matrix) || !cv::checkRange(distortion))
      throw std::runtime_error("calibration returned non-finite results");
    std::vector<double> per_view;
    for(std::size_t i=0;i<image_points.size();++i) {
      std::vector<cv::Point2f> projected;
      cv::projectPoints(object_points[i],rotations[i],translations[i],matrix,distortion,projected);
      double squared=0;
      for(std::size_t j=0;j<projected.size();++j) { const auto delta=projected[j]-image_points[i][j]; squared+=delta.dot(delta); }
      per_view.push_back(std::sqrt(squared/projected.size()));
    }
    YAML::Emitter result;
    result.SetDoublePrecision(17);
    result << YAML::BeginMap << YAML::Key << "image_width" << YAML::Value << image_size.width
      << YAML::Key << "image_height" << YAML::Value << image_size.height
      << YAML::Key << "distortion_model" << YAML::Value << "opencv_pinhole"
      << YAML::Key << "camera_matrix" << YAML::Value << YAML::Flow << std::vector<double>(matrix.begin<double>(),matrix.end<double>())
      << YAML::Key << "distort_coeffs" << YAML::Value << YAML::Flow << std::vector<double>(distortion.begin<double>(),distortion.end<double>())
      << YAML::Key << "fit_rms_px" << YAML::Value << rms << YAML::Key << "pattern" << YAML::Value << pattern
      << YAML::Key << "pattern_cols" << YAML::Value << cols << YAML::Key << "pattern_rows" << YAML::Value << rows
      << YAML::Key << "center_distance_mm" << YAML::Value << spacing_mm
      << YAML::Key << "validation_status" << YAML::Value << "fit_only_requires_independent_check"
      << YAML::Key << "views" << YAML::Value << YAML::BeginSeq;
    for(std::size_t i=0;i<sources.size();++i) result << YAML::BeginMap << YAML::Key << "source" << YAML::Value << sources[i]
      << YAML::Key << "fit_rms_px" << YAML::Value << per_view[i] << YAML::EndMap;
    result << YAML::EndSeq << YAML::EndMap;
    if(!result.good()) throw std::runtime_error("cannot serialize calibration");
    if(!fs::path(output).parent_path().empty()) fs::create_directories(fs::path(output).parent_path());
    std::ofstream file(output);
    file << result.c_str() << '\n'; file.flush();
    if(!file) throw std::runtime_error("cannot write calibration output");
    bof::load_calibration(output);
    std::cout << "Saved " << output << " | views=" << sources.size() << " | fit RMS=" << rms
              << "px. This is fit error, not independent calibration validation.\n";
    return 0;
  } catch(const std::exception& error) { std::cerr << "bof_calibrate: " << error.what() << '\n'; return 1; }
}
