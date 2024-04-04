#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <geometry_msgs/Twist.h>
#include <string>

class CVUIROSCmdVelPublisher {
public:
  CVUIROSCmdVelPublisher();
  void updateControls(cv::Mat& frame);
  
private:
  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_msg_;
  float linear_velocity_step_;
  float angular_velocity_step_;
  std::string window_name_;
  // Other members for your GUI, if any

};

void drawTable(cv::Mat& frame, const std::string& info_message);
void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& msg);

extern std::string info_message;

#endif 
