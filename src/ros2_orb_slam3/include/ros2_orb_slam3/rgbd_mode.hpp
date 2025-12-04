#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/core/core.hpp>

#include "System.h"            // orb_slam3/System
#include "Tracking.h"

class RGBDMode : public rclcpp::Node {
public:
  RGBDMode();

private:
  // Callbacks
  void rgbdCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
                    const sensor_msgs::msg::Image::ConstSharedPtr& depth);

  // ORB-SLAM3
  std::unique_ptr<ORB_SLAM3::System> system_;
  std::string voc_path_;
  std::string settings_path_;
  bool initialized_ = false;

  // Subscribers + sync
  using Image = sensor_msgs::msg::Image;
  message_filters::Subscriber<Image> sub_rgb_;
  message_filters::Subscriber<Image> sub_d_;
  using Policy = message_filters::sync_policies::ApproximateTime<Image,Image>;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;

  // Parámetros y nombres de tópicos
  std::string topic_rgb_, topic_depth_;

  // Bridge
  cv_bridge::CvImageConstPtr cv_ptr_rgb_, cv_ptr_depth_;
};
