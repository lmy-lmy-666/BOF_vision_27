// Camera-to-gimbal hand-eye calibration from a rosbag of board images and gimbal attitude.
// Adapted from Tongji sp_vision_25 calibration/calibrate_handeye.cpp (MIT): gimbal rotation about a
// fixed centre (zero gimbal translation), board PnP, cv::calibrateHandEye. Changes: rosbag input with
// attitude interpolated at the image stamp, still-gimbal and view-diversity selection, chessboard
// support, Tsai initialisation refined by joint reprojection least squares (camera extrinsics plus the
// fixed board pose, idea of Tongji calibrate_robotworld_handeye.cpp), output as aim.extrinsics.
// Tsai alone uses PnP board rotations, which are too noisy for the small swings a fixed board allows.
#include "bof_vision/orientation_buffer.hpp"
#include "bof_vision/pose.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
constexpr double pi=3.14159265358979323846;
struct View {Eigen::Matrix3d gimbal; std::vector<cv::Point2f> points;};
Eigen::Matrix3d rotation(const Eigen::Vector3d& r) {
  return r.norm()<1e-12?Eigen::Matrix3d::Identity():Eigen::AngleAxisd(r.norm(),r.normalized()).toRotationMatrix();
}
// p = [camera->gimbal rvec, camera in gimbal, board->reference rvec, board in reference]
Eigen::VectorXd residuals(const Eigen::Matrix<double,12,1>& p,const std::vector<View>& views,
                          const std::vector<cv::Point3f>& object,const bof::CameraCalibration& camera) {
  const Eigen::Matrix3d Rx=rotation(p.segment<3>(0)), Rw=rotation(p.segment<3>(6));
  Eigen::VectorXd r(views.size()*object.size()*2); int k=0;
  for(const auto& v:views) {
    const Eigen::Matrix3d Rwc=v.gimbal*Rx; const Eigen::Vector3d twc=v.gimbal*p.segment<3>(3);
    const Eigen::Matrix3d Rcb=Rwc.transpose()*Rw; const Eigen::Vector3d tcb=Rwc.transpose()*(p.segment<3>(9)-twc);
    cv::Mat R,rvec,tvec; cv::eigen2cv(Rcb,R); cv::Rodrigues(R,rvec); cv::eigen2cv(tcb,tvec);
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object,rvec,tvec,camera.camera_matrix,camera.distort_coeffs,projected);
    for(std::size_t i=0;i<object.size();++i) {r[k++]=projected[i].x-v.points[i].x; r[k++]=projected[i].y-v.points[i].y;}
  }
  return r;
}
// Levenberg-Marquardt with a numeric Jacobian; 12 parameters, a few hundred residuals.
double refine(Eigen::Matrix<double,12,1>& p,const std::vector<View>& views,const std::vector<cv::Point3f>& object,
              const bof::CameraCalibration& camera) {
  double lambda=1e-3; Eigen::VectorXd r=residuals(p,views,object,camera);
  for(int iteration=0;iteration<100;++iteration) {
    Eigen::MatrixXd J(r.size(),12);
    for(int j=0;j<12;++j) {
      auto q=p; const double h=j%6<3?1e-6:1e-6; q[j]+=h;
      J.col(j)=(residuals(q,views,object,camera)-r)/h;
    }
    const Eigen::MatrixXd A=J.transpose()*J; const Eigen::VectorXd g=J.transpose()*r;
    bool improved=false;
    for(int attempt=0;attempt<10 && !improved;++attempt) {
      Eigen::MatrixXd D=A; D.diagonal()*=1+lambda;
      const Eigen::Matrix<double,12,1> step=D.ldlt().solve(-g);
      const auto q=(p+step).eval(); const auto rq=residuals(q,views,object,camera);
      if(rq.squaredNorm()<r.squaredNorm()) {
        improved=true; lambda=std::max(lambda/10,1e-9);
        const bool done=step.norm()<1e-10; p=q; r=rq; if(done) iteration=100;
      } else lambda*=10;
    }
    if(!improved) break;
  }
  return std::sqrt(r.squaredNorm()/(r.size()/2));
}
template<class T> T decode(const rosbag2_storage::SerializedBagMessage& m) {
  static rclcpp::Serialization<T> serialization; T message;
  rclcpp::SerializedMessage serialized(*m.serialized_data);
  serialization.deserialize_message(&serialized,&message);
  return message;
}
}
int main(int argc,char** argv) {
  try {
    std::string bag, calibration, pattern="circles", image_topic="/camera/image_raw", output="log/artifacts/handeye.yaml";
    int cols=0, rows=0, min_views=10; double spacing_mm=0, still_dps=1.0, distinct_deg=3.0;
    for(int i=1;i<argc;++i) {
      const std::string a=argv[i]; auto next=[&]{if(i+1>=argc) throw std::invalid_argument("missing value for "+a); return std::string(argv[++i]);};
      if(a=="--bag") bag=next(); else if(a=="--calibration") calibration=next(); else if(a=="--pattern") pattern=next();
      else if(a=="--cols") cols=std::stoi(next()); else if(a=="--rows") rows=std::stoi(next());
      else if(a=="--spacing-mm") spacing_mm=std::stod(next()); else if(a=="--image-topic") image_topic=next();
      else if(a=="--still-deg-per-s") still_dps=std::stod(next()); else if(a=="--distinct-deg") distinct_deg=std::stod(next());
      else if(a=="--min-views") min_views=std::stoi(next()); else if(a=="--output") output=next();
      else if(a=="--help" || a=="-h") {
        std::cout << "bof_handeye --bag BAG --calibration camera.yaml --pattern circles|chessboard --cols N --rows N --spacing-mm MM\n"
          "  [--image-topic /camera/image_raw] [--still-deg-per-s 1] [--distinct-deg 3] [--min-views 10] [--output log/artifacts/handeye.yaml]\n"
          "Fix the board, then hold the gimbal still at >=10 yaw/pitch combinations (both axes) while recording\n"
          "  ros2 bag record /camera/image_raw /gimbal/imu\n"
          "cols/rows: inner corners for chessboards, dot centres for circles.\n";
        return 0;
      } else throw std::invalid_argument("unknown argument "+a);
    }
    if(bag.empty() || calibration.empty() || cols<2 || rows<2 || !(spacing_mm>0))
      throw std::invalid_argument("--bag, --calibration, --cols, --rows and --spacing-mm are required");
    if(pattern!="circles" && pattern!="chessboard") throw std::invalid_argument("pattern must be circles or chessboard");
    const auto camera=bof::load_calibration(calibration);
    const cv::Size board(cols,rows);
    std::vector<cv::Point3f> object;
    for(int r=0;r<rows;++r) for(int c=0;c<cols;++c) object.push_back({float(c*spacing_mm/1000),float(r*spacing_mm/1000),0});

    // Attitude first, then images (bag order is not guaranteed to put IMU before each image).
    bof::OrientationBuffer attitude(bof::ClockDomain::ros,50000000,1<<22);
    // Serialized messages are released by the storage plugin, so the reader must outlive them.
    rosbag2_cpp::Reader reader; reader.open(bag);
    std::vector<std::pair<std::int64_t,std::shared_ptr<rosbag2_storage::SerializedBagMessage>>> images;
    {
      std::vector<bof::OrientationSample> samples;
      while(reader.has_next()) {
        auto m=reader.read_next();
        if(m->topic_name=="/gimbal/imu") {
          auto imu=decode<sensor_msgs::msg::Imu>(*m);
          samples.push_back({rclcpp::Time(imu.header.stamp).nanoseconds(),
            Eigen::Quaterniond(imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z)});
        } else if(m->topic_name==image_topic) images.push_back({m->recv_timestamp,m});
      }
      std::sort(samples.begin(),samples.end(),[](const auto& a,const auto& b){return a.stamp_ns<b.stamp_ns;});
      for(const auto& s:samples) attitude.push(s);
    }
    std::cout << "images on " << image_topic << ": " << images.size() << "\n";
    std::vector<cv::Mat> R_gimbal,t_gimbal,R_board,t_board;
    std::vector<Eigen::Quaterniond> used; std::vector<View> views;
    int moving=0, similar=0, not_found=0;
    for(const auto& [recv,m]:images) {
      const auto msg=decode<sensor_msgs::msg::Image>(*m);
      const std::int64_t t=rclcpp::Time(msg.header.stamp).nanoseconds();
      const auto q=attitude.at(t,bof::ClockDomain::ros), before=attitude.at(t-100000000,bof::ClockDomain::ros),
                 after=attitude.at(t+100000000,bof::ClockDomain::ros);
      if(!q || !before || !after) {++moving; continue;}
      // Still gimbal: rotation over +-0.1 s below the threshold, so stamp errors do not matter.
      if(before->angularDistance(*after)*180/pi/0.2>still_dps) {++moving; continue;}
      bool distinct=true;
      for(const auto& u:used) if(u.angularDistance(*q)*180/pi<distinct_deg) {distinct=false; break;}
      if(!distinct) {++similar; continue;}
      cv::Mat gray; cv::cvtColor(cv_bridge::toCvCopy(msg,"bgr8")->image,gray,cv::COLOR_BGR2GRAY);
      if(gray.size()!=camera.image_size) throw std::invalid_argument("image size differs from the calibration file");
      std::vector<cv::Point2f> points;
      const bool found=pattern=="circles"?cv::findCirclesGrid(gray,board,points,cv::CALIB_CB_SYMMETRIC_GRID)
                                         :cv::findChessboardCornersSB(gray,board,points,cv::CALIB_CB_NORMALIZE_IMAGE);
      if(!found) {++not_found; continue;}
      cv::Mat rvec,tvec;
      if(!cv::solvePnP(object,points,camera.camera_matrix,camera.distort_coeffs,rvec,tvec)) {++not_found; continue;}
      cv::Mat R; cv::Rodrigues(rvec,R);
      cv::Mat Rg; cv::eigen2cv(Eigen::Matrix3d(q->toRotationMatrix()),Rg);
      R_gimbal.push_back(Rg); t_gimbal.push_back(cv::Mat::zeros(3,1,CV_64F));
      R_board.push_back(R); t_board.push_back(tvec); used.push_back(*q); views.push_back({q->toRotationMatrix(),points});
    }
    std::cout << "views used " << used.size() << ", skipped moving/no-attitude " << moving << ", similar " << similar
              << ", board not found " << not_found << "\n";
    if(static_cast<int>(used.size())<min_views) throw std::runtime_error("not enough distinct still views; cover more yaw and pitch angles");
    cv::Mat R_cam,t_cam;
    cv::calibrateHandEye(R_gimbal,t_gimbal,R_board,t_board,R_cam,t_cam,cv::CALIB_HAND_EYE_TSAI);
    Eigen::Matrix3d Rc; Eigen::Vector3d tc; cv::cv2eigen(R_cam,Rc); cv::cv2eigen(t_cam,tc);
    // Initial board pose in the reference frame from the first view, then joint refinement.
    Eigen::Matrix3d Rb0; Eigen::Vector3d tb0; cv::cv2eigen(R_board[0],Rb0); cv::cv2eigen(t_board[0],tb0);
    Eigen::Matrix<double,12,1> p;
    p.segment<3>(0)=Eigen::AngleAxisd(Rc).angle()*Eigen::AngleAxisd(Rc).axis(); p.segment<3>(3)=tc;
    const Eigen::Matrix3d Rw0=used[0].toRotationMatrix()*Rc*Rb0;
    p.segment<3>(6)=Eigen::AngleAxisd(Rw0).angle()*Eigen::AngleAxisd(Rw0).axis(); p.segment<3>(9)=used[0]*(Rc*tb0+tc);
    const double initial_rms=std::sqrt(residuals(p,views,object,camera).squaredNorm()/(views.size()*object.size()));
    const double rms=refine(p,views,object,camera);
    std::cout << "reprojection RMS: Tsai " << initial_rms << " px -> refined " << rms << " px\n";
    Rc=rotation(p.segment<3>(0)); tc=p.segment<3>(3);
    const Eigen::Quaterniond qc(Rc);
    // The board is fixed, so its reference-frame pose should be the same in every view.
    std::vector<Eigen::Vector3d> centres; double angle_sum=0; Eigen::Matrix3d first;
    for(std::size_t i=0;i<used.size();++i) {
      Eigen::Matrix3d Rb; Eigen::Vector3d tb; cv::cv2eigen(R_board[i],Rb); cv::cv2eigen(t_board[i],tb);
      const Eigen::Matrix3d world=used[i].toRotationMatrix()*Rc*Rb;
      centres.push_back(used[i]*(Rc*tb+tc));
      if(i==0) first=world; else angle_sum+=Eigen::AngleAxisd(first.transpose()*world).angle();
    }
    Eigen::Vector3d mean=Eigen::Vector3d::Zero(); for(const auto& c:centres) mean+=c; mean/=centres.size();
    double spread=0; for(const auto& c:centres) spread=std::max(spread,(c-mean).norm());
    // Mount angles relative to the ideal camera (optical z along gimbal +x, x along -y, y along -z).
    Eigen::Matrix3d ideal; ideal<<0,0,1,-1,0,0,0,-1,0;
    const double mount_deg=Eigen::AngleAxisd(ideal.transpose()*Rc).angle()*180/pi;
    YAML::Emitter out;
    out << YAML::BeginMap
        << YAML::Key << "camera_to_gimbal_q_xyzw" << YAML::Value << YAML::Flow << std::vector<double>{qc.x(),qc.y(),qc.z(),qc.w()}
        << YAML::Key << "camera_in_gimbal_m" << YAML::Value << YAML::Flow << std::vector<double>{tc.x(),tc.y(),tc.z()}
        << YAML::Key << "views" << YAML::Value << used.size()
        << YAML::Key << "reprojection_rms_px" << YAML::Value << rms
        << YAML::Key << "board_centre_max_spread_m" << YAML::Value << spread
        << YAML::Key << "board_orientation_mean_spread_deg" << YAML::Value << (used.size()>1?angle_sum/(used.size()-1)*180/pi:0)
        << YAML::Key << "deviation_from_ideal_mount_deg" << YAML::Value << mount_deg
        << YAML::Key << "validation_status" << YAML::Value << "consistency_only_requires_aim_check" << YAML::EndMap;
    const auto output_parent = std::filesystem::path(output).parent_path();
    if (!output_parent.empty()) std::filesystem::create_directories(output_parent);
    std::ofstream(output) << "# Copy the two transforms into aim.extrinsics. Large spreads mean bad attitude sync or views.\n"
                          << out.c_str() << '\n';
    std::cout << out.c_str() << "\nSaved " << output << "\n";
  } catch(const std::exception& e) {std::cerr << "bof_handeye: " << e.what() << "\n"; return 1;}
  return 0;
}
