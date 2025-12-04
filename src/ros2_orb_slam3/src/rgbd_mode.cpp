#include "ros2_orb_slam3/rgbd_mode.hpp"
#include <rclcpp/parameter.hpp>
#include <opencv2/imgproc/imgproc.hpp>

RGBDMode::RGBDMode() : rclcpp::Node("orb_slam3_rgbd_node"),
  sub_rgb_(this, "camera/color/image_raw"),      // default: puedes cambiar por param
  sub_d_  (this, "camera/depth/image_raw") {

  // Parámetros
  this->declare_parameter<std::string>("voc_path", "ORBvoc.txt");
  this->declare_parameter<std::string>("settings_path", "orb_slam3/config/RGB-D/RealSense_D435i.yaml");
  this->declare_parameter<std::string>("topic_rgb", "camera/color/image_raw");
  this->declare_parameter<std::string>("topic_depth", "camera/depth/image_raw");
  this->declare_parameter<double>("depth_scale", 1000.0);
double depth_scale = this->get_parameter("depth_scale").as_double();

  this->get_parameter("voc_path", voc_path_);
  this->get_parameter("settings_path", settings_path_);
  this->get_parameter("topic_rgb", topic_rgb_);
  this->get_parameter("topic_depth", topic_depth_);

  // Configurar subs con los tópicos parametrizados
  sub_rgb_.subscribe(this, topic_rgb_);
  sub_d_.subscribe(this, topic_depth_);

  sync_ = std::make_unique<message_filters::Synchronizer<Policy>>(Policy(10), sub_rgb_, sub_d_);
  sync_->registerCallback(std::bind(&RGBDMode::rgbdCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 RGBD node starting...\n  voc: %s\n  settings: %s\n  rgb: %s\n  depth: %s",
              voc_path_.c_str(), settings_path_.c_str(), topic_rgb_.c_str(), topic_depth_.c_str());

  // Iniciar ORB-SLAM3 en modo RGBD
  system_ = std::make_unique<ORB_SLAM3::System>(voc_path_, settings_path_,
             ORB_SLAM3::System::RGBD, true /*use viewer*/);
}

void RGBDMode::rgbdCallback(const Image::ConstSharedPtr& rgb,
                            const Image::ConstSharedPtr& depth) {
  try {
    cv_ptr_rgb_   = cv_bridge::toCvShare(rgb, "bgr8");
    // depth: puede venir en 16UC1 (milímetros) o 32FC1 (metros)
    if (depth->encoding == "16UC1") {
      cv_ptr_depth_ = cv_bridge::toCvShare(depth, "16UC1");
    } else {
      cv_ptr_depth_ = cv_bridge::toCvShare(depth, "32FC1");
    }
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    return;
  }

  // ORB-SLAM3 espera depth en metros (CV_32F). Convertimos si llega en 16UC1 (mm).
  cv::Mat depth_m;
  if (cv_ptr_depth_->image.type() == CV_16UC1) {
    // usa el parámetro
    double scale = this->get_parameter("depth_scale").as_double();
    cv_ptr_depth_->image.convertTo(depth_m, CV_32F, 1.0 / scale);
  } else {
    depth_m = cv_ptr_depth_->image; // ya CV_32F en metros
  }

  // Timestamp
  const double tframe = rclcpp::Time(rgb->header.stamp).seconds();

  // TrackRGBD (BGR8 + depth float32 metros)
  system_->TrackRGBD(cv_ptr_rgb_->image, depth_m, tframe);
}

