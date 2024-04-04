#include "ros/ros.h"
#include <std_msgs/String.h>
#include "robot_gui/robot_gui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h" 
#include <opencv2/opencv.hpp>

#include "robot_gui/cvui.h"

#define WINDOW_NAME "Robot GUI"

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_gui_node");
    if (!ros::master::check()) {
        ROS_ERROR("roscore is not running!");
        return -1;
    }

    ros::NodeHandle nh;
    cv::Mat frame = cv::Mat(400, 600, CV_8UC3);
    cv::namedWindow(WINDOW_NAME);

    cvui::init(WINDOW_NAME);
    
    ros::Subscriber sub = nh.subscribe("robot_info", 1000, robotInfoCallback);
    ROS_INFO("robot_gui_node started and subscribing to 'robot_info'.");

    while (ros::ok()) {
        frame = cv::Scalar(49, 52, 49); 
        drawTable(frame, info_message);;

        
        cvui::update();
        cv::imshow(WINDOW_NAME, frame);
        
        /
        ros::spinOnce();
        int key = cv::waitKey(20);
        if (key == 27) { 
            break;
        }
    }
    ROS_INFO("Exiting robot_gui_node.");
    return 0;
}
