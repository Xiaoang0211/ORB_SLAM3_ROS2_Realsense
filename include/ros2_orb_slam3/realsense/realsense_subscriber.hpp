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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

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
    double lastTimeStep = 0.0;

    RealsenseSubscriber(std::unordered_map<std::string, std::string>& options);
    ~RealsenseSubscriber();

    rmw_qos_profile_t qos_profile;

private:
    std::string homeDir = "";
    std::string packagePath = "orbslam3_ws/src/ros2_orb_slam3";
    std::string nodeName = "";
    std::string vocFilePath = "";
    std::string settingsFilePath = "";
    std::string realsenseConfigFile = "";
    std::string mode = "";

    // sensor topic names
    std::string rgbImageTopic = "";
    std::string depthImageTopic = "";
    std::string accelTopic = "";
    std::string gyroTopic = "";

    // subscribers
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgMsg_subscription_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> accel_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> gyro_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_image_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_image_sub_;

    // imu buffer
    std::vector<ORB_SLAM3::IMU::Point> imu_buffer_;
    std::mutex imu_mutex_;

    // message synchronizer
    using imuSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Imu,
                                                                 sensor_msgs::msg::Imu>;
    std::shared_ptr<message_filters::Synchronizer<imuSyncPolicy>> imu_sync_;

    using imageSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                                      sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<imageSyncPolicy>> image_sync_;

    // callback for synchronized messages
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_image_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_image_msg);

    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& accel_msg,
                     const sensor_msgs::msg::Imu::ConstSharedPtr& gyro_msg);

    // // callback for subscription
    // void ImgCallback(const sensor_msgs::msg::Image& msg);


    // ORB_SLAM3
    ORB_SLAM3::System* pAgent;
    ORB_SLAM3::System::eSensor sensorType;
    bool enablePangolinWindow = false;
    bool enableOpenCVWindow = false;


    // Helper functions
    template<typename SensorType>
    double get_time_ns_double(const SensorType& msg)
    {
        return msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec;
    }
    void InitializeVSLAM(std::string& configString);
    ORB_SLAM3::System::eSensor convertStringToSensorType(const std::string& str);

};