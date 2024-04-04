#include "ros/ros.h"
#include <std_msgs/String.h>
#include "robot_gui/robot_gui.h"
#include <geometry_msgs/Twist.h>
#include "robotinfo_msgs/RobotInfo10Fields.h" 
#include <opencv2/opencv.hpp>


#define CVUI_DISABLE_COMPILATION_NOTICES
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#define WINDOW_NAME "Robot GUI"

std::string info_message;

void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& msg) {
    
    std::stringstream ss;
    ss << msg->data_field_01 << "\n"; 
    ss << msg->data_field_02 << "\n"; 
    ss << msg->data_field_03 << "\n"; 
    ss << msg->data_field_04 << "\n";
    ss << msg->data_field_05 << "\n";
    ss << msg->data_field_06 << "\n";
    ss << msg->data_field_07 << "\n";
    ss << msg->data_field_08 << "\n";
    ss << msg->data_field_09 << "\n"; 
    ss << msg->data_field_10 << "\n";

    
    info_message = ss.str();
}

  void drawTable(cv::Mat& frame, const std::string& info_message) {
    std::istringstream f(info_message);
    std::string line;
    std::vector<std::string> lines;
    while (std::getline(f, line)) {
        lines.push_back(line);
    }
    
    int tableTop = 20;
    int titleHeight = 30;
    int tableHeight = 30 * lines.size();

   
    cvui::text(frame, 15, tableTop + 8, "Info", 0.7, 0x000000);
    
    tableTop += titleHeight;

    cv::rectangle(frame, cv::Point(10, tableTop), cv::Point(frame.cols - 10, tableTop + tableHeight), CV_RGB(49, 52, 49), cv::FILLED);

    int y = tableTop;

    for (const auto& current_line : lines) { 
        size_t colonPos = current_line.find(":");
        if (colonPos != std::string::npos) {
            std::string label = current_line.substr(0, colonPos + 1);         
               std::string value = current_line.substr(colonPos + 2);

            cvui::text(frame, 15, y + 8, label, 0.4, 0xFFFFFF);
             cvui::text(frame, 200, y + 8, value, 0.4, 0xFFFFFF);
        }

        y += 30;
     }
}

CVUIROSCmdVelPublisher::CVUIROSCmdVelPublisher()
: linear_velocity_step_(0.1), angular_velocity_step_(0.1), window_name_("CVUI ROS TELEOP") {
  ros::NodeHandle nh;
  twist_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  
}

void CVUIROSCmdVelPublisher::updateControls(cv::Mat& frame) {
     
  if (cvui::button(frame, 100, 300, "Forward")) {
        this->twist_msg_.linear.x += this->linear_velocity_step_;
        this->twist_pub_.publish(this->twist_msg_);
    }
    
    if (cvui::button(frame, 100, 300, " Forward ")) {
      twist_msg_.linear.x = twist_msg_.linear.x + linear_velocity_step_;
      twist_pub_.publish(twist_msg_);
    }

    if (cvui::button(frame, 100, 330, "   Stop  ")) {
      twist_msg_.linear.x = 0.0;
      twist_msg_.angular.z = 0.0;
      twist_pub_.publish(twist_msg_);
    }

    if (cvui::button(frame, 30, 330, " Left ")) {
      twist_msg_.angular.z = twist_msg_.angular.z + angular_velocity_step_;
      twist_pub_.publish(twist_msg_);
    }

    if (cvui::button(frame, 195, 330, " Right ")) {
      twist_msg_.angular.z = twist_msg_.angular.z - angular_velocity_step_;
      twist_pub_.publish(twist_msg_);
    }

    if (cvui::button(frame, 100, 360, "Backward")) {
      twist_msg_.linear.x = twist_msg_.linear.x - linear_velocity_step_;
      twist_pub_.publish(twist_msg_);
    }
    
    cvui::window(frame, 30, 400, 120, 40, "Linear velocity:");
    cvui::printf(frame, 55, 425, 0.4, 0xff0000, "%.02f m/sec",
                 twist_msg_.linear.x);

    cvui::window(frame, 150, 400, 120, 40, "Angular velocity:"); 
    cvui::printf(frame, 175, 425, 0.4, 0xff0000, "%.02f rad/sec",
                 twist_msg_.angular.z);

    
}
