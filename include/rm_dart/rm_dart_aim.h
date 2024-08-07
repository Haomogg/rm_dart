//
// Created by ywj on 24-6-1.
//

#ifndef RM_DART_AIM_H
#define RM_DART_AIM_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rm_dart/DartConfig.h>
#include <image_transport/image_transport.h>
#include <rm_msgs/Dart.h>
#include "rm_msgs/DartClientCmd.h"
#include <rm_common/filters/filters.h>
#include <shared_mutex>
#include <boost/filesystem.hpp>

enum class DrawImage
{
  DISABLE = 0,
  RAW = 1,
  BINARY = 2,
  MORPHOLOGY = 3,
  TARGET = 4
};

namespace rm_dart {
class RmDartAim : public nodelet::Nodelet
{
public:
  void onInit() override;
  void callback(const sensor_msgs::CompressedImageConstPtr& image);
  void configCB(DartConfig& config);
  bool imageProcess(cv_bridge::CvImagePtr& cv_image);
  void draw();
  void dartClientCmdCallBack(const rm_msgs::DartClientCmd& msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  ros::Subscriber dart_client_cmd_sub_;
  ros::Publisher image_pub_;
  ros::Publisher dart_pub_;

  bool is_open_;
  bool is_save_;
//  cv_bridge::CvImagePtr cv_image_;
  dynamic_reconfigure::Server<rm_dart::DartConfig>* server_;
  dynamic_reconfigure::Server<rm_dart::DartConfig>::CallbackType callback_;

  int green_h_min_;
  int green_h_max_;
  int green_s_min_;
  int green_s_max_;
  int green_v_min_;
  int green_v_max_;

  int image_x_center_;

  cv::Mat raw_image_;
  cv::Mat binary_image_;
  cv::Mat morpro_image_;
  cv::Mat target_image_;

  cv::Point target_center_;
  DrawImage draw_type_;
  cv::VideoWriter writer;

  cv::RotatedRect rect_;

  std::shared_ptr<MovingAverageFilter<float>> pos_buffer_;
  int buffer_num_;
  int max_distance_;

  int current_num_ = 0;

  std::string save_path_;
  int image_num_ = 0;
};
}


#endif //RM_DART_AIM_H
