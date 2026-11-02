// Field-debug entry point using the shared perception and camera adapters.
// Recording is adapted from Tongji; latest buffering is adapted from SZURPVision.
#include "bof_vision/camera.hpp"
#include "bof_vision/config.hpp"
#include "bof_vision/latest_buffer.hpp"
#include "bof_vision/pipeline.hpp"
#include "bof_vision/pipeline_switcher.hpp"
#include "bof_vision/recorder.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {
namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
volatile std::sig_atomic_t interrupted=0;
void on_signal(int) { interrupted=1; }
struct Input { bof::Frame frame; Clock::time_point received; std::uint64_t generation=0; };
double number(const std::string& value) {
  std::size_t used=0;
  const double result=std::stod(value,&used);
  if(used!=value.size() || !std::isfinite(result)) throw std::invalid_argument("invalid number: "+value);
  return result;
}
int integer(const std::string& value) {
  std::size_t used=0;
  const int result=std::stoi(value,&used);
  if(used!=value.size()) throw std::invalid_argument("invalid integer: "+value);
  return result;
}
std::string run_name() {
  const auto now=std::chrono::system_clock::now();
  const auto seconds=std::chrono::system_clock::to_time_t(now);
  std::tm local{}; localtime_r(&seconds,&local);
  std::ostringstream out;
  out << "live_" << std::put_time(&local,"%Y%m%d_%H%M%S") << '_'
      << std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count()%1000000;
  return out.str();
}
const char* label_name(bof::ArmorLabel label) {
  switch(label) {
    case bof::ArmorLabel::sentry: return "sentry";
    case bof::ArmorLabel::hero: return "hero";
    case bof::ArmorLabel::engineer: return "engineer";
    case bof::ArmorLabel::infantry3: return "infantry3";
    case bof::ArmorLabel::infantry4: return "infantry4";
    case bof::ArmorLabel::infantry5: return "infantry5";
    case bof::ArmorLabel::outpost: return "outpost";
    case bof::ArmorLabel::base: return "base";
    default: return "unknown";
  }
}
void write_yaml(const fs::path& path,const YAML::Emitter& content) {
  std::ofstream output(path);
  output << content.c_str() << '\n'; output.flush();
  if(!content.good() || !output) throw std::runtime_error("cannot write "+path.string());
}
}

int main(int argc,char** argv) {
  try {
    std::string config_path,video_path,camera_serial,output_path;
    int usb=-1,limit=0,retry_delay_ms=1000;
    bool synthetic=false,headless=false,start_recording=false,reconnect=true;
    double fps=0,record_fps=30,exposure=3000,gain=0,reconnect_timeout_s=30;
    std::vector<std::pair<std::string,std::string>> options;
    for(int i=1;i<argc;++i) {
      const std::string key=argv[i];
      if(key=="--help") {
        std::cout << "bof_live (--camera auto|SERIAL | --usb INDEX | --video PATH | --synthetic)\n"
          " [--config PATH] [--yolo PROFILE] [--backend NAME --model PATH --device CPU --color red|blue]\n"
          " [--exposure-us 3000 --gain 0] [--fps N] [--frames N] (0 = until quit/end of video)\n"
          " [--record --record-fps 30] [--output NEW_DIRECTORY] [--headless]\n"
          " [--calibration PATH --plate-width-m W --plate-height-m H --pose-label ID]\n"
          " [--calibration PATH --geometry PATH] (multi-label small/large geometry policies)\n"
          " [--no-reconnect] [--reconnect-timeout-s 30] [--retry-delay-ms 1000] (timeout 0 = unlimited)\n"
          " [--track-confirm-frames 3 --track-label-confirm-frames 3 --association-ambiguity-px 12]\n"
          "Keys: q/Esc quit, y next model profile, r record on/off, s snapshot, b red/blue, +/- confidence,\n"
          "      [/] exposure x0.8/x1.25, g/G gain -1/+1 (Hik only). Ctrl-C also exits.\n";
        return 0;
      }
      if(key=="--headless" || key=="--window" || key=="--record" || key=="--no-record" ||
         key=="--synthetic" || key=="--no-reconnect" || key=="--reconnect") { options.emplace_back(key,""); continue; }
      if(i+1>=argc) throw std::invalid_argument("missing value for "+key);
      const std::string value=argv[++i];
      if(key=="--config") config_path=value;
      else options.emplace_back(key,value);
    }
    auto settings=bof::load_config(config_path);
    for(const auto& [key,value]:options) if(key=="--yolo") bof::select_yolo(settings,value);
    const auto& runtime=settings.live;
    headless=runtime.headless; start_recording=runtime.record; reconnect=runtime.reconnect;
    limit=runtime.frames; fps=runtime.fps; record_fps=runtime.record_fps;
    exposure=runtime.exposure_us; gain=runtime.gain; reconnect_timeout_s=runtime.reconnect_timeout_s;
    retry_delay_ms=runtime.retry_delay_ms; output_path=runtime.output;
    int explicit_sources=0;
    for(const auto& [key,value]:options) if(key=="--camera" || key=="--video" || key=="--usb" || key=="--synthetic") ++explicit_sources;
    if(explicit_sources>1) throw std::invalid_argument("choose one explicit input source");
    if(!explicit_sources) {
      if(runtime.source=="hik") camera_serial=runtime.camera_serial;
      else if(runtime.source=="video") video_path=runtime.video_path;
      else if(runtime.source=="usb") usb=runtime.usb_index;
      else if(runtime.source=="synthetic") synthetic=true;
    }
    for(const auto& [key,value]:options) {
      if(key=="--yolo") continue;
      else if(key=="--synthetic") synthetic=true;
      else if(key=="--headless") headless=true;
      else if(key=="--window") headless=false;
      else if(key=="--record") start_recording=true;
      else if(key=="--no-record") start_recording=false;
      else if(key=="--no-reconnect") reconnect=false;
      else if(key=="--reconnect") reconnect=true;
      else if(key=="--video") video_path=fs::absolute(value).string();
      else if(key=="--camera") camera_serial=value;
      else if(key=="--usb") usb=integer(value);
      else if(key=="--frames") limit=integer(value);
      else if(key=="--fps") fps=number(value);
      else if(key=="--record-fps") record_fps=number(value);
      else if(key=="--exposure-us") exposure=number(value);
      else if(key=="--gain") gain=number(value);
      else if(key=="--reconnect-timeout-s") reconnect_timeout_s=number(value);
      else if(key=="--retry-delay-ms") retry_delay_ms=integer(value);
      else if(key=="--output") output_path=value;
      else if(key=="--backend") { settings.detector.backend=value; settings.yolo_name.clear(); }
      else if(key=="--model") { settings.detector.model_path=value; settings.yolo_name.clear(); }
      else if(key=="--device") settings.detector.device=value;
      else if(key=="--color") settings.detector.enemy_color=bof::parse_color(value);
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
      else throw std::invalid_argument("unknown option: "+key);
    }
    if(static_cast<int>(!video_path.empty())+!camera_serial.empty()+(usb>=0)+synthetic!=1)
      throw std::invalid_argument("choose exactly one source: --camera, --usb, --video or --synthetic");
    if(limit<0 || exposure<=0 || gain<0 || record_fps<=0 || record_fps>1000 || fps<0 || fps>1000 || (fps>0 && fps<1)
       || reconnect_timeout_s<0 || retry_delay_ms<50)
      throw std::invalid_argument("invalid frame limit, exposure, gain or FPS");
    if(!headless && !std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY") && !std::getenv("QT_QPA_PLATFORM"))
      throw std::runtime_error("no desktop display; use --headless for SSH/console");
    settings.live.source=!camera_serial.empty() ? "hik" : usb>=0 ? "usb" : synthetic ? "synthetic" : "video";
    settings.live.video_path=video_path; settings.live.camera_serial=camera_serial;
    if(usb>=0) settings.live.usb_index=usb;
    settings.live.frames=limit; settings.live.fps=fps; settings.live.record_fps=record_fps;
    settings.live.exposure_us=exposure; settings.live.gain=gain;
    settings.live.headless=headless; settings.live.record=start_recording; settings.live.reconnect=reconnect;
    settings.live.reconnect_timeout_s=reconnect_timeout_s; settings.live.retry_delay_ms=retry_delay_ms;
    settings.live.output=output_path.empty() ? "" : fs::absolute(output_path).string();
    bof::validate_config(settings);
    if(!camera_serial.empty() && !bof::hik_camera_enabled()) throw std::runtime_error("Hik camera requires BOF_WITH_MVS=ON; rebuild before connecting");
    bof::PipelineSwitcher pipeline(settings);
    cv::VideoCapture video;
    if(!video_path.empty() && !video.open(video_path)) throw std::runtime_error("cannot open video: "+video_path);
    if(fps==0) fps=!video_path.empty() ? video.get(cv::CAP_PROP_FPS) : 30.0;
    if(!std::isfinite(fps) || fps<1 || fps>1000) throw std::invalid_argument("unknown video FPS; set --fps explicitly");
    const bool hardware=!camera_serial.empty() || usb>=0;
    const std::string source=!camera_serial.empty() ? "hik" : usb>=0 ? "usb" : synthetic ? "synthetic" : "video";
    const std::string time_domain=hardware ? "host_receive" : "nominal_video";
    const fs::path output=fs::absolute(output_path.empty() ? (fs::path("artifacts/output")/run_name()).string() : output_path);
    fs::create_directories(output.parent_path());
    if(!fs::create_directory(output)) throw std::runtime_error("output directory already exists: "+output.string());
    bof::save_config(settings,(output/"initial_config.yaml").string());
    bof::save_config(settings,(output/"effective_config.yaml").string());
    if(!settings.pose.calibration_path.empty()) fs::copy_file(settings.pose.calibration_path,output/"calibration_used.yaml");
    if(!settings.pose.geometry_path.empty()) fs::copy_file(settings.pose.geometry_path,output/"geometry_used.yaml");
    YAML::Emitter session;
    session << YAML::BeginMap << YAML::Key << "source" << YAML::Value << source
      << YAML::Key << "video" << YAML::Value << video_path << YAML::Key << "camera_serial" << YAML::Value << camera_serial
      << YAML::Key << "usb_index" << YAML::Value << usb << YAML::Key << "source_clock" << YAML::Value << time_domain
      << YAML::Key << "requested_exposure_us" << YAML::Value << exposure
      << YAML::Key << "requested_gain" << YAML::Value << gain
      << YAML::Key << "nominal_playback_fps" << YAML::Value << fps
      << YAML::Key << "record_fps" << YAML::Value << record_fps
      << YAML::Key << "reconnect" << YAML::Value << reconnect
      << YAML::Key << "reconnect_timeout_s" << YAML::Value << reconnect_timeout_s
      << YAML::Key << "retry_delay_ms" << YAML::Value << retry_delay_ms
      << YAML::Key << "recording" << YAML::Value << "sampled processed raw frames; use each frames.csv for actual timestamps"
      << YAML::EndMap;
    write_yaml(output/"session.yaml",session);
    std::ofstream metrics(output/"observations.csv"),events(output/"events.csv");
    if(!metrics || !events) throw std::runtime_error("cannot open session logs");
    metrics << "sequence,stamp_ns,candidates,top_label,top_confidence,process_ms,receive_to_result_ms,overwritten,capture_generation,track_valid,track_id,track_status,track_label,observed_label,candidate_index,association_error_px,model_revision,yolo_name\n";
    std::ofstream poses(output/"poses.csv");
    if(!poses) throw std::runtime_error("cannot open pose log");
    poses << "sequence,stamp_ns,candidate_index,label,valid,status,x_m,y_m,z_m,distance_m,rx_rad,ry_rad,rz_rad,reprojection_rms_px,alternative_rms_px,ambiguous,geometry,geometry_source,width_m,height_m,image_ratio,capture_generation,model_revision\n";
    events << "after_sequence,event,value\n";
    std::atomic<bool> stop{false};
    std::atomic<int> synthetic_color{settings.detector.enemy_color==bof::Color::red ? 0 : 1};
    std::atomic<double> wanted_exposure{exposure},wanted_gain{gain},applied_exposure{0},applied_gain{-1};
    std::atomic<std::uint64_t> produced{0};
    std::atomic<bool> capture_connected{!hardware};
    std::atomic<unsigned> retries{0};
    std::mutex capture_status_mutex;
    std::string capture_status="waiting for first frame",selected_serial;
    std::ofstream capture_log(output/"capture.log");
    if(!capture_log) throw std::runtime_error("cannot open capture log");
    bof::LatestBuffer<Input> latest;
    std::exception_ptr capture_error;
    std::unique_ptr<bof::Recorder> recorder;
    unsigned recording_index=0;
    auto toggle_recording=[&](std::uint64_t sequence) {
      if(recorder) {
        recorder->close();
        std::cout << "Recording stopped: saved=" << recorder->saved() << " overwritten=" << recorder->overwritten() << '\n';
        events << sequence << ",record_stop," << recorder->saved() << '\n';
        recorder.reset();
      } else {
        const auto directory=output/("recording_"+std::to_string(++recording_index));
        recorder=std::make_unique<bof::Recorder>(directory.string(),record_fps);
        settings.live.record=true;
        bof::save_config(settings,(directory/"config_at_start.yaml").string());
        events << sequence << ",record_start," << recording_index << '\n';
        std::cout << "Recording raw frames to " << directory << '\n';
      }
      settings.live.record=static_cast<bool>(recorder);
      pipeline.set_live_parameters(settings.live);
      bof::save_config(settings,(output/"effective_config.yaml").string());
    };
    if(start_recording) toggle_recording(0);
    const std::string window="BOF Vision | q quit | y model | r record | s snapshot | b color | +/- confidence | [ ] exposure | g G gain";
    if(!headless) cv::namedWindow(window,cv::WINDOW_NORMAL);
    std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal);
    std::cout << "source=" << source << " backend=" << settings.detector.backend << " output=" << output
              << "\n" << bof::config_yaml(settings) << std::flush;
    std::thread producer([&] {
      // Capture restart flow adapted from Tongji HikRobot's daemon: release then reopen.
      // Keep recovery on this thread; UI and inference never call SDK functions concurrently.
      try {
        std::unique_ptr<bof::CameraSource> camera;
        std::string pinned_serial=camera_serial=="auto" ? "" : camera_serial;
        double last_exposure=exposure,last_gain=gain;
        unsigned consecutive_timeouts=0;
        std::uint64_t generation=0;
        bool first_frame=true;
        std::optional<Clock::time_point> outage_start;
        auto next=Clock::now();
        while(!stop && !interrupted && (limit==0 || produced<static_cast<std::uint64_t>(limit))) {
          try {
            if(!camera_serial.empty() && !camera) {
              const double requested_exposure=wanted_exposure.load(),requested_gain=wanted_gain.load();
              camera=bof::make_hik_camera(pinned_serial,requested_exposure,requested_gain);
              if(pinned_serial.empty()) pinned_serial=camera->serial_number();
              selected_serial=pinned_serial;
              applied_exposure=camera->set_exposure_us(requested_exposure);
              applied_gain=camera->set_gain(requested_gain);
              last_exposure=requested_exposure; last_gain=requested_gain;
              ++generation; first_frame=true; consecutive_timeouts=0;
            } else if(usb>=0 && !video.isOpened()) {
              if(!video.open(usb,cv::CAP_V4L2)) throw std::runtime_error("cannot open USB camera index "+std::to_string(usb));
              ++generation; first_frame=true;
            }
            cv::Mat image;
            auto received=Clock::now();
            std::uint64_t sequence=produced;
            if(camera) {
              const double requested_exposure=wanted_exposure.load(),requested_gain=wanted_gain.load();
              if(requested_exposure!=last_exposure) {
                applied_exposure=camera->set_exposure_us(requested_exposure); last_exposure=requested_exposure;
              }
              if(requested_gain!=last_gain) { applied_gain=camera->set_gain(requested_gain); last_gain=requested_gain; }
              auto frame=camera->read(200);
              if(!frame) {
                if(++consecutive_timeouts>=10) throw std::runtime_error("camera produced no frames for 10 reads");
                continue;
              }
              consecutive_timeouts=0;
              image=std::move(frame->bgr); sequence=frame->sequence; received=frame->received;
            } else if(synthetic) {
              image=bof::synthetic_frame(static_cast<int>(produced.load()%1000000),synthetic_color==0 ? bof::Color::red : bof::Color::blue);
              received=Clock::now();
            } else {
              if(!video.read(image) || image.empty()) {
                if(usb>=0) throw std::runtime_error("USB camera read failed");
                break;
              }
              received=Clock::now();
            }
            if(hardware && first_frame) {
              capture_log << "connected generation=" << generation << " serial=" << selected_serial << '\n' << std::flush;
              { std::lock_guard<std::mutex> lock(capture_status_mutex); capture_status="connected"; }
              first_frame=false; outage_start.reset(); capture_connected=true;
            }
            const auto stamp=hardware ? std::chrono::duration_cast<std::chrono::nanoseconds>(received.time_since_epoch()).count()
              : static_cast<std::int64_t>(std::llround(produced.load()*1e9/fps));
            ++produced;
            if(!latest.push({{image,stamp,sequence,"camera_optical_frame",hardware ? bof::ClockDomain::host_receive : bof::ClockDomain::nominal_video},received,generation})) break;
            if(!hardware) {
              next+=std::chrono::nanoseconds(static_cast<std::int64_t>(1e9/fps));
              while(!stop && !interrupted && Clock::now()<next) std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
          } catch(const std::exception& error) {
            if(!hardware) throw;
            capture_connected=false; latest.discard(); camera.reset();
            if(usb>=0) video.release();
            { std::lock_guard<std::mutex> lock(capture_status_mutex); capture_status=error.what(); }
            capture_log << "failure attempt=" << ++retries << " message=" << error.what() << '\n' << std::flush;
            std::cerr << "Capture unavailable: " << error.what() << "; attempt=" << retries.load() << '\n';
            if(!reconnect) throw;
            if(!outage_start) outage_start=Clock::now();
            if(reconnect_timeout_s>0 && Clock::now()-*outage_start>=std::chrono::duration<double>(reconnect_timeout_s))
              throw std::runtime_error("camera reconnect deadline exceeded; last error: "+std::string(error.what()));
            const auto retry_at=Clock::now()+std::chrono::milliseconds(retry_delay_ms);
            while(!stop && !interrupted && Clock::now()<retry_at) std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        }
      } catch(...) { capture_error=std::current_exception(); capture_connected=false; }
      latest.close();
    });
    auto shutdown=[&] { stop=true; latest.close(); if(producer.joinable()) producer.join(); };
    std::uint64_t processed=0,last_reported=0,last_sequence=0;
    std::uint64_t active_generation=0;
    std::string model_cursor=settings.yolo_name;
    bool waiting_for_camera=false;
    auto last_report=Clock::now();
    double display_fps=0;
    cv::Mat last_canvas;
    try {
      while(!interrupted) {
        const auto switched=pipeline.poll();
        if(switched==bof::SwitchResult::applied) {
          settings=pipeline.config();
          bof::save_config(settings,(output/"effective_config.yaml").string());
          bof::save_config(settings,(output/("config_model_revision_"+std::to_string(pipeline.revision())+".yaml")).string());
          events << last_sequence << ",model_applied," << settings.yolo_name << '\n' << std::flush;
          std::cout << "Model applied: " << settings.yolo_name << " cache=" << pipeline.cache_hit() << '\n';
        } else if(switched==bof::SwitchResult::failed) {
          events << last_sequence << ",model_switch_failed,retained_previous\n" << std::flush;
          std::cerr << "Model switch failed; current model retained: " << pipeline.error() << '\n';
        }
        auto input=latest.wait_pop_for(std::chrono::milliseconds(100));
        if(!input || (hardware && !capture_connected)) {
          if(latest.closed()) break;
          if(hardware && !capture_connected && !waiting_for_camera) {
            pipeline.reset(); waiting_for_camera=true;
            events << last_sequence << ",camera_wait," << retries.load() << '\n' << std::flush;
          }
          if(recorder) recorder->check_error();
          if(!headless) {
            cv::Mat waiting(480,800,CV_8UC3,cv::Scalar(0,0,0));
            std::string reason;
            { std::lock_guard<std::mutex> lock(capture_status_mutex); reason=capture_status; }
            cv::putText(waiting,"WAITING FOR FRAME  retries="+std::to_string(retries.load()),{20,80},cv::FONT_HERSHEY_SIMPLEX,0.7,{0,200,255},2);
            cv::putText(waiting,reason.substr(0,90),{20,125},cv::FONT_HERSHEY_SIMPLEX,0.45,{220,220,220},1);
            cv::putText(waiting,"q / Esc to stop; Ctrl-C also exits",{20,170},cv::FONT_HERSHEY_SIMPLEX,0.55,{220,220,220},1);
            cv::imshow(window,waiting);
            const int key=cv::waitKey(1)&0xff;
            if(key=='q' || key==27 || cv::getWindowProperty(window,cv::WND_PROP_VISIBLE)<1) break;
          }
          continue;
        }
        if(hardware && (waiting_for_camera || input->generation!=active_generation)) {
          pipeline.reset();
          if(recorder && active_generation!=0) { toggle_recording(last_sequence); toggle_recording(input->frame.sequence); }
          active_generation=input->generation; waiting_for_camera=false;
          events << input->frame.sequence << ",camera_generation," << active_generation << '\n' << std::flush;
        }
        const auto result=pipeline.process(input->frame);
        last_sequence=result.sequence; ++processed;
        const double age=std::chrono::duration<double,std::milli>(Clock::now()-input->received).count();
        int best_label=-1; float best_confidence=-1;
        if(!result.candidates.empty()) {
          const auto best=std::max_element(result.candidates.begin(),result.candidates.end(),[](const auto& a,const auto& b){return a.score<b.score;});
          best_label=static_cast<int>(best->label); best_confidence=best->confidence;
        }
        metrics << std::setprecision(10) << result.sequence << ',' << result.stamp_ns << ',' << result.candidates.size()
          << ',' << best_label << ',' << best_confidence << ',' << result.process_ms << ',' << age << ',' << latest.overwritten() << ',' << input->generation
          << ',' << result.track.valid << ',' << result.track.id << ',' << result.track.status << ',' << static_cast<int>(result.track.label)
          << ',' << static_cast<int>(result.track.observed_label) << ',' << result.track.candidate_index << ',' << result.track.association_error_px
          << ',' << pipeline.revision() << ',' << settings.yolo_name << '\n';
        if(recorder) recorder->record(input->frame.bgr,result.sequence,result.stamp_ns,input->received);
        for(std::size_t i=0;i<result.poses.size();++i) {
          const auto& p=result.poses[i];
          poses << std::setprecision(10) << result.sequence << ',' << result.stamp_ns << ',' << i << ','
            << static_cast<int>(result.candidates[i].label) << ',' << p.valid << ',' << p.status << ','
            << p.position_m[0] << ',' << p.position_m[1] << ',' << p.position_m[2] << ',' << p.distance_m << ','
            << p.rotation_vector[0] << ',' << p.rotation_vector[1] << ',' << p.rotation_vector[2] << ','
            << p.reprojection_error_px << ',' << p.alternative_error_px << ',' << p.ambiguous << ','
            << p.geometry << ',' << p.geometry_source << ',' << p.width_m << ',' << p.height_m << ',' << p.image_ratio << ',' << input->generation << ',' << pipeline.revision() << '\n';
        }
        auto canvas=input->frame.bgr.clone();
        for(std::size_t i=0;i<result.candidates.size();++i) {
          const auto& candidate=result.candidates[i];
          for(int j=0;j<4;++j) {
            cv::line(canvas,candidate.corners[j],candidate.corners[(j+1)%4],{0,255,0},2);
            cv::circle(canvas,candidate.corners[j],3,{0,255,255},-1);
          }
          std::ostringstream label;
          label << label_name(candidate.label);
          if(candidate.size_hint!=bof::PlateSize::unknown) label << '/' << bof::plate_size_name(candidate.size_hint);
          if(candidate.confidence>=0) label << ' ' << std::fixed << std::setprecision(2) << candidate.confidence;
          if(!result.poses.empty()) {
            const auto& p=result.poses[i];
            if(p.valid) label << " [" << p.geometry << "] d=" << std::fixed << std::setprecision(2) << p.distance_m << "m err="
              << p.reprojection_error_px << "px" << (p.ambiguous ? " AMBIG" : "");
            else if(p.status!="label_filtered") label << " PnP:" << p.status;
          }
          cv::putText(canvas,label.str(),candidate.corners[0],cv::FONT_HERSHEY_SIMPLEX,0.55,{0,255,255},2);
        }
        if(result.track.valid) cv::circle(canvas,result.track.center,6,{255,255,0},2);
        const auto now=Clock::now();
        display_fps=(processed-last_reported)/std::max(1e-6,std::chrono::duration<double>(now-last_report).count());
        if(now-last_report>=std::chrono::seconds(1)) {
          last_reported=processed; last_report=now;
          std::cout << "processed=" << processed << " fps=" << std::fixed << std::setprecision(1) << display_fps
            << " candidates=" << result.candidates.size() << " core_ms=" << result.process_ms
            << " track=" << result.track.status << " id=" << result.track.id
            << " dropped=" << latest.overwritten() << " recording=" << (recorder ? "on" : "off") << '\n' << std::flush;
          metrics.flush(); events.flush(); poses.flush();
          if(!metrics || !events || !poses) throw std::runtime_error("session log write failed");
        }
        std::ostringstream status;
        status << settings.detector.backend << " enemy=" << bof::color_name(settings.detector.enemy_color)
          << " fps=" << std::fixed << std::setprecision(1) << display_fps << " core=" << result.process_ms
          << "ms age=" << age << "ms drop=" << latest.overwritten() << (recorder ? " REC" : "");
        if(pipeline.busy()) status << " loading=" << pipeline.pending_name();
        else if(!pipeline.error().empty()) status << " SWITCH FAILED (see log)";
        cv::rectangle(canvas,{0,0},{canvas.cols,78},{0,0,0},-1);
        cv::putText(canvas,status.str(),{8,21},cv::FONT_HERSHEY_SIMPLEX,0.48,{255,255,255},1);
        std::ostringstream controls;
        controls << "threshold=" << std::setprecision(2) << settings.detector.confidence_threshold;
        if(!camera_serial.empty()) controls << " exposure=" << std::fixed << std::setprecision(0) << applied_exposure.load() << "us gain=" << applied_gain.load();
        controls << " | " << time_domain << " | q quit, y model, r record, s snapshot, b color";
        if(settings.pose.calibration_path.empty()) controls << " | PnP off";
        cv::putText(canvas,controls.str(),{8,44},cv::FONT_HERSHEY_SIMPLEX,0.45,{255,255,255},1);
        const auto& track=result.track;
        const std::string association="track="+track.status+" id="+std::to_string(track.id)+
          " stable="+label_name(track.label)+" raw="+label_name(track.observed_label)+
          " candidate="+std::to_string(track.candidate_index);
        cv::putText(canvas,association,{8,67},cv::FONT_HERSHEY_SIMPLEX,0.45,{0,220,255},1);
        last_canvas=canvas;
        int key=-1;
        if(!headless) {
          cv::imshow(window,canvas);
          key=cv::waitKey(1)&0xff;
          if(cv::getWindowProperty(window,cv::WND_PROP_VISIBLE)<1) break;
        }
        if(key=='q' || key==27) break;
        if(key=='y' && !pipeline.busy() && !settings.yolo_models.empty()) {
          auto next=settings.yolo_models.upper_bound(model_cursor);
          if(next==settings.yolo_models.end()) next=settings.yolo_models.begin();
          model_cursor=next->first;  // A failed/uninstalled model must not trap the cycle here.
          if(pipeline.request(next->first)) events << result.sequence << ",model_requested," << next->first << '\n' << std::flush;
          else {
            events << result.sequence << ",model_switch_failed," << next->first << '\n' << std::flush;
            std::cerr << "Model switch rejected; current model retained: " << pipeline.error() << '\n';
          }
        }
        if(key=='r') toggle_recording(result.sequence);
        if(key=='s') {
          const auto stem=output/("snapshot_g"+std::to_string(input->generation)+"_"+std::to_string(result.sequence));
          if(!cv::imwrite(stem.string()+"_raw.png",input->frame.bgr) || !cv::imwrite(stem.string()+"_overlay.png",canvas))
            throw std::runtime_error("snapshot write failed");
          events << result.sequence << ",snapshot,1\n";
        }
        if(!pipeline.busy() && (key=='b' || key=='+' || key=='=' || key=='-')) {
          auto changed=settings;
          if(key=='b') changed.detector.enemy_color=settings.detector.enemy_color==bof::Color::red ? bof::Color::blue : bof::Color::red;
          else changed.detector.confidence_threshold=std::clamp(settings.detector.confidence_threshold+(key=='-' ? -0.05F : 0.05F),0.05F,0.95F);
          pipeline.set_detection_parameters(changed.detector);
          settings=pipeline.config();
          synthetic_color=settings.detector.enemy_color==bof::Color::red ? 0 : 1;
          bof::save_config(settings,(output/"effective_config.yaml").string());
          bof::save_config(settings,(output/("config_after_g"+std::to_string(input->generation)+"_"+std::to_string(result.sequence)+".yaml")).string());
          events << result.sequence << ",config_changed," << result.sequence << '\n';
        }
        if(!camera_serial.empty() && (key=='[' || key==']')) {
          wanted_exposure=std::max(1.0,applied_exposure.load()*(key=='[' ? 0.8 : 1.25));
          settings.live.exposure_us=wanted_exposure.load(); pipeline.set_live_parameters(settings.live);
          bof::save_config(settings,(output/"effective_config.yaml").string());
          events << result.sequence << ",requested_exposure_us," << wanted_exposure.load() << '\n';
        }
        if(!camera_serial.empty() && (key=='g' || key=='G')) {
          wanted_gain=std::max(0.0,applied_gain.load()+(key=='g' ? -1 : 1));
          settings.live.gain=wanted_gain.load(); pipeline.set_live_parameters(settings.live);
          bof::save_config(settings,(output/"effective_config.yaml").string());
          events << result.sequence << ",requested_gain," << wanted_gain.load() << '\n';
        }
      }
    } catch(...) { shutdown(); throw; }
    shutdown();
    if(recorder) toggle_recording(last_sequence);
    if(!headless) cv::destroyAllWindows();
    if(!last_canvas.empty() && !cv::imwrite((output/"last_overlay.png").string(),last_canvas)) throw std::runtime_error("cannot save final preview");
    metrics.flush(); events.flush(); poses.flush();
    if(!metrics || !events || !poses) throw std::runtime_error("session log write failed");
    YAML::Emitter summary;
    summary << YAML::BeginMap << YAML::Key << "produced" << YAML::Value << produced.load()
      << YAML::Key << "processed" << YAML::Value << processed << YAML::Key << "overwritten" << YAML::Value << latest.overwritten()
      << YAML::Key << "applied_exposure_us" << YAML::Value << applied_exposure.load()
      << YAML::Key << "applied_gain" << YAML::Value << applied_gain.load()
      << YAML::Key << "capture_retries" << YAML::Value << retries.load()
      << YAML::Key << "capture_generation" << YAML::Value << active_generation
      << YAML::Key << "selected_camera_serial" << YAML::Value << selected_serial
      << YAML::Key << "capture_failed" << YAML::Value << static_cast<bool>(capture_error) << YAML::EndMap;
    write_yaml(output/"summary.yaml",summary);
    if(capture_error) std::rethrow_exception(capture_error);
    if(processed==0 && !interrupted) throw std::runtime_error("no frames processed");
    std::cout << "Stopped: produced=" << produced << " processed=" << processed << " overwritten=" << latest.overwritten()
      << "\nSession: " << output << '\n';
    return 0;
  } catch(const std::exception& error) {
    std::cerr << "bof_live: " << error.what() << '\n'; return 1;
  }
}
