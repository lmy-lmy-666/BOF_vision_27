#include "bof_vision/pipeline.hpp"
#include "bof_vision/latest_buffer.hpp"
#include "bof_vision/camera.hpp"
#include "bof_vision/config.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {
struct Input { bof::Frame frame; std::chrono::steady_clock::time_point received; };
int integer(const std::string& value) {
  std::size_t end;
  int result=std::stoi(value,&end);
  if(end!=value.size()) throw std::invalid_argument("invalid integer: "+value);
  return result;
}
double number(const std::string& value) {
  std::size_t end;
  double result=std::stod(value,&end);
  if(end!=value.size() || !std::isfinite(result)) throw std::invalid_argument("invalid number: "+value);
  return result;
}
double percentile(std::vector<double> values,double p) {
  if(values.empty()) return 0;
  std::sort(values.begin(),values.end());
  return values.at(static_cast<std::size_t>(std::ceil(p*values.size()))-1);
}
}
int main(int argc,char** argv) {
  try {
    std::string config_path,snapshot_path;
    bool check_config=false;
    std::vector<std::pair<std::string,std::string>> options;
    std::string video_path,csv_path,detections_path,preview_path,mode="all",camera_serial;
    int limit=100,slow_ms=0;
    double fps=0,exposure_us=3000;
    for(int i=1;i<argc;++i) {
      std::string key=argv[i];
      if(key=="--help") {
        std::cout << "bof_replay [--video PATH] [--frames N] [--fps N] [--color red|blue]\n"
          " [--backend light_pair|tongji_yolov5|tongji_yolov8|tongji_yolo11|shenzhen_0708|shenzhen_0526] [--model PATH] [--classifier tiny_resnet.onnx] [--device CPU]\n"
          " [--mode all|latest] [--csv PATH] [--preview PATH.png] [--slow-ms N]\n"
          " [--detections PATH.jsonl] (all candidates and TL/TR/BR/BL corners, including empty frames)\n"
          " [--camera auto|SERIAL --exposure-us N] (optional MVS build)\n"
          " [--config PATH.yaml] [--dump-config PATH.yaml] [--check-config]\n"
          " [--yolo PROFILE] (select backend/weights/device/thresholds from the YAML catalogue)\n"
          " [--calibration PATH --plate-width-m W --plate-height-m H --pose-label ID]\n"
          " [--calibration PATH --geometry PATH] (multi-label geometry policies)\n"
          " [--color-threshold N] [--confidence N] [--nms N] [--track-timeout-s N] [--association-gate-px N]\n"
          " [--track-confirm-frames 3] [--track-label-confirm-frames 3] [--association-ambiguity-px 12]\n"
          "Explicit options override YAML regardless of argument order. --check-config exits before opening inputs/inference.\n";
        return 0;
      }
      if(key=="--check-config") { check_config=true; continue; }
      if(i+1>=argc) throw std::invalid_argument("missing value for "+key);
      std::string value=argv[++i];
      if(key=="--config") config_path=value;
      else options.emplace_back(key,value);
    }
    auto settings=bof::load_config(config_path);
    for(const auto& [key,value]:options) if(key=="--yolo") bof::select_yolo(settings,value);
    auto& config=settings.detector;
    for(const auto& option:options) {
      const auto& key=option.first;
      const auto& value=option.second;
      if(key=="--yolo") continue;
      else if(key=="--video") video_path=value;
      else if(key=="--camera") camera_serial=value;
      else if(key=="--exposure-us") exposure_us=number(value);
      else if(key=="--frames") limit=integer(value);
      else if(key=="--fps") fps=number(value);
      else if(key=="--backend") { config.backend=value; settings.yolo_name.clear(); }
      else if(key=="--model") { config.model_path=value; settings.yolo_name.clear(); }
      else if(key=="--device") config.device=value;
      else if(key=="--classifier") config.classifier_path=value;
      else if(key=="--csv") csv_path=value;
      else if(key=="--detections") detections_path=value;
      else if(key=="--preview") preview_path=value;
      else if(key=="--mode") mode=value;
      else if(key=="--slow-ms") slow_ms=integer(value);
      else if(key=="--color") config.enemy_color=bof::parse_color(value);
      else if(key=="--dump-config") snapshot_path=value;
      else if(key=="--color-threshold") config.color_threshold=integer(value);
      else if(key=="--confidence") config.confidence_threshold=number(value);
      else if(key=="--nms") config.nms_threshold=number(value);
      else if(key=="--track-timeout-s") settings.track_timeout_s=number(value);
      else if(key=="--association-gate-px") settings.association_gate_px=number(value);
      else if(key=="--track-confirm-frames") settings.association.confirm_frames=integer(value);
      else if(key=="--track-label-confirm-frames") settings.association.label_confirm_frames=integer(value);
      else if(key=="--association-ambiguity-px") settings.association.ambiguity_px=number(value);
      else if(key=="--calibration") settings.pose.calibration_path=value;
      else if(key=="--geometry") settings.pose.geometry_path=value;
      else if(key=="--plate-width-m") settings.pose.plate_width_m=number(value);
      else if(key=="--plate-height-m") settings.pose.plate_height_m=number(value);
      else if(key=="--pose-label") settings.pose.label=integer(value);
      else if(key=="--max-reprojection-error-px") settings.pose.max_reprojection_error_px=number(value);
      else throw std::invalid_argument("unknown option "+key);
    }
    if(limit<1 || slow_ms<0 || (mode!="all" && mode!="latest")) throw std::invalid_argument("invalid replay options");
    if(fps!=0 && (fps<1 || fps>1000)) throw std::invalid_argument("fps must be 0 (auto) or in [1,1000]");
    if(exposure_us<=0) throw std::invalid_argument("exposure-us must be positive");
    cv::VideoCapture video;
    if(!camera_serial.empty() && !video_path.empty()) throw std::invalid_argument("choose camera or video, not both");
    bof::validate_config(settings);
    std::cout << "Effective perception configuration:\n" << bof::config_yaml(settings);
    if(snapshot_path.empty() && !csv_path.empty()) snapshot_path=csv_path+".config.yaml";
    if(snapshot_path.empty() && !detections_path.empty()) snapshot_path=detections_path+".config.yaml";
    if(!snapshot_path.empty()) bof::save_config(settings,snapshot_path);
    if(check_config) return 0;
    std::unique_ptr<bof::CameraSource> camera;
    if(!video_path.empty() && !video.open(video_path)) throw std::runtime_error("cannot open video: "+video_path);
    if(fps==0) fps=video_path.empty() ? 30.0 : video.get(cv::CAP_PROP_FPS);
    if(!std::isfinite(fps) || fps<1 || fps>1000) throw std::invalid_argument("invalid fps; specify --fps");
    bof::PerceptionPipeline pipeline(config,settings.track_timeout_s,settings.association_gate_px,settings.pose,settings.association);
    // Compile inference before acquisition starts, so initialization does not fill the SDK queue.
    if(!camera_serial.empty()) camera=bof::make_hik_camera(camera_serial=="auto" ? "" : camera_serial,exposure_us);
    std::ofstream csv;
    std::ofstream detections;
    if(!detections_path.empty()) {
      detections.open(detections_path);
      if(!detections) throw std::runtime_error("cannot write detections: "+detections_path);
    }
    if(!csv_path.empty()) {
      csv.open(csv_path);
      if(!csv) throw std::runtime_error("cannot write CSV: "+csv_path);
      csv << "sequence,stamp_ns,candidates,valid,track_id,label,u,v,detect_ms,process_ms,source_to_result_ms,top_label,top_confidence,track_status,observed,observed_label,candidate_index,association_error_px\n";
    }
    int produced=0,processed=0,with_candidates=0;
    std::vector<double> process_times,ages;
    cv::Mat last_preview;
    auto read=[&]() -> std::optional<Input> {
      if(produced>=limit) return std::nullopt;
      cv::Mat img;
      if(camera) {
        auto captured=camera->read();
        if(!captured) throw std::runtime_error("camera timeout; check connection and restart capture");
        ++produced;
        bof::Frame frame{captured->bgr,std::chrono::duration_cast<std::chrono::nanoseconds>(
          captured->received.time_since_epoch()).count(),captured->sequence,"camera_optical_frame",bof::ClockDomain::host_receive};
        return Input{std::move(frame),captured->received};
      }
      if(video_path.empty()) img=bof::synthetic_frame(produced,config.enemy_color);
      else if(!video.read(img) || img.empty()) return std::nullopt;
      bof::Frame frame{img,static_cast<std::int64_t>(std::llround(produced*1e9/fps)),
        static_cast<std::uint64_t>(produced),"camera_optical_frame",bof::ClockDomain::nominal_video};
      ++produced;
      return Input{std::move(frame),std::chrono::steady_clock::now()};
    };
    auto consume=[&](const Input& input) {
      if(slow_ms) std::this_thread::sleep_for(std::chrono::milliseconds(slow_ms));
      auto result=pipeline.process(input.frame);
      double age=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-input.received).count();
      ++processed;
      if(!result.candidates.empty()) ++with_candidates;
      int top_label=-1; float top_confidence=-1;
      if(!result.candidates.empty()) {
        const auto best=std::max_element(result.candidates.begin(),result.candidates.end(),
          [](const auto& a,const auto& b){return a.score<b.score;});
        top_label=static_cast<int>(best->label); top_confidence=best->confidence;
      }
      if(processed>5) { process_times.push_back(result.process_ms); ages.push_back(age); }
      if(csv) csv << std::setprecision(12) << result.sequence << ',' << result.stamp_ns << ','
        << result.candidates.size() << ',' << result.track.valid << ',' << result.track.id << ','
        << static_cast<int>(result.track.label) << ',' << result.track.center.x << ',' << result.track.center.y << ','
        << result.detect_ms << ',' << result.process_ms << ',' << age << ',' << top_label << ',' << top_confidence
        << ',' << result.track.status << ',' << result.track.observed << ',' << static_cast<int>(result.track.observed_label)
        << ',' << result.track.candidate_index << ',' << result.track.association_error_px << '\n';
      if(detections.is_open()) {
        detections << std::setprecision(9) << "{\"sequence\":" << result.sequence
          << ",\"stamp_ns\":" << result.stamp_ns << ",\"width\":" << input.frame.bgr.cols
          << ",\"height\":" << input.frame.bgr.rows << ",\"candidates\":[";
        for(std::size_t i=0;i<result.candidates.size();++i) {
          const auto& candidate=result.candidates[i];
          if(i) detections << ',';
          detections << "{\"label\":" << static_cast<int>(candidate.label)
            << ",\"raw_class\":" << candidate.raw_class << ",\"size_hint\":\"" << bof::plate_size_name(candidate.size_hint) << '"'
            << ",\"confidence\":" << candidate.confidence << ",\"score\":" << candidate.score << ",\"corners\":[";
          for(std::size_t j=0;j<4;++j) {
            if(j) detections << ',';
            detections << '[' << candidate.corners[j].x << ',' << candidate.corners[j].y << ']';
          }
          detections << ']';
          if(!result.poses.empty()) {
            const auto& p=result.poses[i];
            detections << ",\"pose\":{\"valid\":" << (p.valid ? "true" : "false") << ",\"status\":\"" << p.status
              << "\",\"position_m\":[" << p.position_m[0] << ',' << p.position_m[1] << ',' << p.position_m[2]
              << "],\"rotation_vector\":[" << p.rotation_vector[0] << ',' << p.rotation_vector[1] << ',' << p.rotation_vector[2]
              << "],\"distance_m\":" << p.distance_m << ",\"reprojection_rms_px\":" << p.reprojection_error_px
              << ",\"alternative_rms_px\":" << p.alternative_error_px << ",\"ambiguous\":" << (p.ambiguous ? "true" : "false")
              << ",\"geometry\":\"" << p.geometry << "\",\"geometry_source\":\"" << p.geometry_source
              << "\",\"width_m\":" << p.width_m << ",\"height_m\":" << p.height_m << ",\"image_ratio\":" << p.image_ratio << '}';
          }
          detections << '}';
        }
        const auto& t=result.track;
        detections << "],\"track\":{\"valid\":" << (t.valid ? "true" : "false")
          << ",\"observed\":" << (t.observed ? "true" : "false") << ",\"id\":" << t.id
          << ",\"status\":\"" << t.status << "\",\"label\":" << static_cast<int>(t.label)
          << ",\"observed_label\":" << static_cast<int>(t.observed_label) << ",\"candidate_index\":" << t.candidate_index
          << ",\"u\":" << t.center.x << ",\"v\":" << t.center.y << ",\"association_error_px\":" << t.association_error_px << "}}\n";
      }
      if(!preview_path.empty()) {
        last_preview=input.frame.bgr.clone();
        for(const auto& candidate:result.candidates) {
          for(int j=0;j<4;++j) cv::line(last_preview,candidate.corners[j],candidate.corners[(j+1)%4],{0,255,0},2);
          const std::string label=std::to_string(static_cast<int>(candidate.label))+" score="+std::to_string(candidate.score).substr(0,4);
          cv::putText(last_preview,label,candidate.center(),cv::FONT_HERSHEY_SIMPLEX,0.5,{0,255,255},1);
        }
      }
    };
    std::uint64_t overwritten=0;
    if(mode=="all") {
      while(auto input=read()) consume(*input);
    } else {
      bof::LatestBuffer<Input> latest;
      std::exception_ptr producer_error;
      std::thread producer([&] {
        try {
          auto next=std::chrono::steady_clock::now();
          while(auto input=read()) {
            if(!latest.push(std::move(*input))) break;
            if(!camera) {
              next+=std::chrono::nanoseconds(static_cast<long long>(1e9/fps));
              std::this_thread::sleep_until(next);
            }
          }
        } catch(...) { producer_error=std::current_exception(); }
        latest.close();
      });
      try { while(auto input=latest.wait_pop()) consume(*input); }
      catch(...) { latest.close(); producer.join(); throw; }
      producer.join(); overwritten=latest.overwritten();
      if(producer_error) std::rethrow_exception(producer_error);
    }
    if(processed==0) throw std::runtime_error("no frames decoded");
    if(csv.is_open()) { csv.flush(); if(!csv) throw std::runtime_error("CSV write failed"); }
    if(detections.is_open()) { detections.flush(); if(!detections) throw std::runtime_error("detections write failed"); }
    if(!preview_path.empty() && !cv::imwrite(preview_path,last_preview)) throw std::runtime_error("cannot write preview");
    std::cout << "backend=" << config.backend << " mode=" << mode << " produced=" << produced
      << " processed=" << processed << " overwritten=" << overwritten << " frames_with_candidates=" << with_candidates
      << "\nafter_first_5_frames samples=" << process_times.size() << " process_p50_ms=" << percentile(process_times,0.50)
      << " process_p95_ms=" << percentile(process_times,0.95) << " source_to_result_p95_ms=" << percentile(ages,0.95)
      << "\nTime source=" << (camera ? "host_receive" : "nominal_video")
      << "; no exposure/IMU synchronization or aiming output. PnP=" << (settings.pose.calibration_path.empty() ? "off" : "camera-optical observations") << '\n';
    return 0;
  } catch(const std::exception& e) { std::cerr << "bof_replay: " << e.what() << '\n'; return 1; }
}
