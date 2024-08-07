//
// Created by ywj on 24-6-1.
//

#include "rm_dart/rm_dart_aim.h"
#include <ros/package.h>

PLUGINLIB_EXPORT_CLASS(rm_dart::RmDartAim, nodelet::Nodelet)

namespace rm_dart
{
void RmDartAim::onInit()
{
  nh_ = this->getMTPrivateNodeHandle();

  nh_.getParam("green_h_max", green_h_max_);
  nh_.getParam("green_h_min", green_h_min_);
  nh_.getParam("green_s_max", green_s_max_);
  nh_.getParam("green_s_min", green_s_min_);
  nh_.getParam("green_v_max", green_v_max_);
  nh_.getParam("green_v_min", green_v_min_);
  nh_.getParam("is_open", is_open_);
  nh_.getParam("is_save", is_save_);

  nh_.getParam("image_x_center", image_x_center_);
  nh_.getParam("buffer_num", buffer_num_);
  nh_.getParam("max_distance", max_distance_);

  server_ = new dynamic_reconfigure::Server<DartConfig>(ros::NodeHandle(nh_, "dart_condition"));
  callback_ = boost::bind(&RmDartAim::configCB, this, _1);
  server_->setCallback(callback_);

  pos_buffer_ = std::make_shared<MovingAverageFilter<float>>(buffer_num_);

  if (is_save_) {
    time_t now = time(nullptr);
    std::string curr_time = ctime(&now);
    curr_time.pop_back();
    save_path_ = ros::package::getPath("rm_dart") + "/vision/" + curr_time;
    boost::filesystem::create_directories(save_path_);
  }

  img_sub_ = nh_.subscribe("/hk_camera/camera/image_raw/compressed", 1, &RmDartAim::callback, this);
  dart_client_cmd_sub_ = nh_.subscribe("/rm_referee/dart_client_cmd_data", 1, &RmDartAim::dartClientCmdCallBack, this);

  image_pub_ = nh_.advertise<sensor_msgs::Image>("/dart_result", 1);
  dart_pub_ = nh_.advertise<rm_msgs::Dart>("/dart_camera_distance", 1);
  ROS_INFO("Dart vision init done");
}

void RmDartAim::callback(const sensor_msgs::CompressedImageConstPtr& image)
{
  if (is_open_)
  {
    cv_bridge::CvImagePtr cv_image_ =
        cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    if (is_save_) {
      cv::imwrite(save_path_ + "/" + std::to_string(++image_num_) + ".jpg", cv_image_->image.clone());
    }
    int yd = cv_image_->image.rows / 3;
    cv_image_->image = cv_image_->image(cv::Rect(0, yd, cv_image_->image.cols, cv_image_->image.rows - yd));

    if (imageProcess(cv_image_))
    {
      int distance = target_center_.x - image_x_center_;
//      std::cout << target_center_.x << std::endl;
//      std::cout << target_center_.y << std::endl;
//      if (abs(distance) > max_distance_)
//        distance = distance / abs(distance) * max_distance_;
      pos_buffer_->input(distance);
      if (current_num_++ >= buffer_num_) {
        rm_msgs::Dart dart_msg;
        dart_msg.distance = pos_buffer_->output();
        dart_msg.stamp = ros::Time::now();
        dart_pub_.publish(dart_msg);
      }
    }
    else
    {
      target_center_.x = -1;
      rm_msgs::Dart dart_msg;
      dart_msg.distance = 0;
      dart_msg.stamp = ros::Time::now();
      dart_pub_.publish(dart_msg);
    }
    draw();
  }
}

void RmDartAim::configCB(DartConfig &config)
{
  draw_type_ = DrawImage(config.draw_type);

  green_h_max_ = config.green_h_max;
  green_h_min_ = config.green_h_min;
  green_s_max_ = config.green_s_max;
  green_s_min_ = config.green_s_min;
  green_v_max_ = config.green_s_max;
  green_v_min_ = config.green_v_min;
}

bool RmDartAim::imageProcess(cv_bridge::CvImagePtr &cv_image)
{
  cv::Mat img = cv_image->image.clone();
  img.copyTo(raw_image_);

  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(-1, -1));

  cv::cvtColor(img, img, cv::COLOR_RGB2HSV);
  cv::inRange(img, cv::Scalar(35, 80, 46), cv::Scalar(77, 255, 255), img);
  img.copyTo(binary_image_);

  cv::morphologyEx(img, img, cv::MORPH_CLOSE, element);
  img.copyTo(morpro_image_);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  unsigned long contours_size = contours.size();
  for (size_t i = 0; i < contours_size; i++)
  {
    if (contours[i].size() < 5)
      continue;
    if (cv::contourArea(contours[i]) < 10)
      continue;

    rect_ = cv::fitEllipse(contours[i]);
    float ratio = float(rect_.size.width) / float(rect_.size.height);
    if (ratio < 1.1 && ratio > 0.9)
    {
      target_center_ = rect_.center;
      return true;
    }
  }
  return false;
}

void RmDartAim::draw()
{
  cv::Mat draw_image;
  sensor_msgs::ImagePtr msg;
  if (draw_type_ == DrawImage::DISABLE)
    return;
  else if (draw_type_ == DrawImage::RAW)
  {
    raw_image_.copyTo(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", draw_image).toImageMsg();
  }
  else if (draw_type_ == DrawImage::BINARY)
  {
    binary_image_.copyTo(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", draw_image).toImageMsg();
  }
  else if (draw_type_ == DrawImage::MORPHOLOGY)
  {
    morpro_image_.copyTo(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", draw_image).toImageMsg();
  }
  else if (draw_type_ == DrawImage::TARGET)
  {
    raw_image_.copyTo(target_image_);
    if (target_center_.x != -1) {
      cv::ellipse(target_image_, rect_, cv::Scalar(0, 0, 255), 2);

      cv::circle(target_image_, rect_.center, 2, cv::Scalar(0, 0, 255), 2, 8,
                 0);
    }
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", target_image_).toImageMsg();
  }
  image_pub_.publish(msg);
}

void RmDartAim::dartClientCmdCallBack(const rm_msgs::DartClientCmd &msg)
{
  if (msg.dart_launch_opening_status == rm_msgs::DartClientCmd::OPENED)
   is_open_ = true;
  else
    is_open_ = false;
}
}
