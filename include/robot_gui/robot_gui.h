#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>

class CVUIROSCmdVelPublisher {
public:
  CVUIROSCmdVelPublisher();
  void updateControls(cv::Mat& frame);
  void updateGUI(cv::Mat& frame);
  
private:
  ros::NodeHandle nh_;
  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_msg_;
  ros::Subscriber odom_sub_; 
  nav_msgs::Odometry current_odom_; 
  float linear_velocity_step_;
  float angular_velocity_step_;
  std::string window_name_;
  // Other members for your GUI, if any
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void drawTable(cv::Mat& frame, const std::string& info_message);
void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& msg);

extern std::string info_message;

#endif 
