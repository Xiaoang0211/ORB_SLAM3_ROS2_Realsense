#pragma once

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <cstdlib>

#include <cstring>
#include <sstream>
// ROS2
#include <rclcpp/rclcpp.hpp>

// ROS2 messages
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
using std::placeholders::_1;

// Eigen
#include <Eigen/Dense>

// cv bridge
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

// ORB SLAM 3 includes
#include "System.h"

#define pass (void)0 // Python's equivalent of "pass", i.e. no operation

class RealsenseSubscriber : public rclcpp::Node
{
public:
    double timeStep;

    RealsenseSubscriber(std::unordered_map<std::string, std::string>& options);
    ~RealsenseSubscriber();
private:
    std::string homeDir = "";
    std::string packagePath = "orbslam3_ws/src/ros2_orb_slam3";
    std::string nodeName = "";
    std::string vocFilePath = "";
    std::string settingsFilePath = "";
    std::string realsenseConfigFile = "";
    std::string subImgMsgName = "";

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgMsg_subscription_;

    // ORB_SLAM3
    ORB_SLAM3::System* pAgent;
    ORB_SLAM3::System::eSensor sensorType;
    bool enablePangolinWindow = false;
    bool enableOpenCVWindow = false;

    // callback for subscription
    void ImgCallback(const sensor_msgs::msg::Image& msg);

    // Helper functions
    template<typename SensorType>
    double get_time_ns_double(const SensorType& msg)
    {
        return msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec;
    }
    void InitializeVSLAM(std::string& configString);
    ORB_SLAM3::System::eSensor convertStringToSensorType(const std::string& str);

};