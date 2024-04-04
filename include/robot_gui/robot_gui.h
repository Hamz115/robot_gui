#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#include <opencv2/opencv.hpp>
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <string>


void drawTable(cv::Mat& frame, const std::string& info_message);
void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& msg);

extern std::string info_message;

#endif 
