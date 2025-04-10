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
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

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

class ReplicaSubscriber : public rclcpp::Node
{
public:
    double timeStep;


    ReplicaSubscriber(std::unordered_map<std::string, std::string>& options);
    ~ReplicaSubscriber();

    rmw_qos_profile_t qos_profile;

private:
    std::string homeDir = "";
    std::string packagePath = "orbslam3_ws/src/ros2_orb_slam3";
    std::string nodeName = "";
    std::string vocFilePath = "";
    std::string settingsFilePath = "";
    std::string ConfigFile = "";
    std::string mode = "";

    // handshake topics
    std::string relicaDriverTopic = "";
    std::string ackTopic = "";

    // sensor topics
    std::string rgbImageTopic = "";
    std::string depthImageTopic = "";

    bool replica_py_driver_activated;

    // handshake publisher and subscriber
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ack_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replica_py_driver_sub_;
    
    // sensor subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_image_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_image_sub_;

    // message synchronizer for rgb and depth images
    using imageSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                                      sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<imageSyncPolicy>> image_sync_;

    // callback for node handshake
    void replica_driver_callback(const std_msgs::msg::Bool& msg);

    // callback for synchronized sensor messages
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_image_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_image_msg);
    
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