/**
 * @file realsense_subscriber.cpp
 * @author Xiaoang Zhang (jesse1008611@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-02-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <ros2_orb_slam3/realsense/realsense_subscriber.hpp>
#include <unordered_map>

RealsenseSubscriber::RealsenseSubscriber(std::unordered_map<std::string, std::string>& options) :
Node("realsense_orbslam_node")
{   
    // default settings for sensors and operating mode
    realsenseConfigFile = "RealSense_D435i.yaml";
    sensorType = ORB_SLAM3::System::eSensor::MONOCULAR;

    if (!options["config"].empty()) {
        realsenseConfigFile = (options.at("config") == "1") ? "RealSense_D435i.yaml" : options["config"];  
    } else if (!options["c"].empty())  {
        realsenseConfigFile = (options.at("c") == "1") ? "RealSense_D435i.yaml" : options["c"];
    }


    if (!options["sensor_type"].empty()) {
        sensorType = (options.at("sensor_type") == "1") ? ORB_SLAM3::System::eSensor::MONOCULAR : convertStringToSensorType(options["sensor_type"]);
    } else if (!options["s"].empty()) {
        sensorType = (options.at("s") == "1") ? ORB_SLAM3::System::eSensor::MONOCULAR : convertStringToSensorType(options["s"]);
    }

    homeDir = getenv("HOME");
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    //* Watchdog, populate default values
    nodeName = "file_not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // set config file path for realsense camera
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        // TODO: add more modes than just monocular
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    InitializeVSLAM(realsenseConfigFile);

    //* DEBUG print 
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

    subImgMsgName = "/camera/camera/color/image_raw"; // rgb image topic specified by realsense ros2

    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subImgMsgName, 1, std::bind(&RealsenseSubscriber::ImgCallback, this, _1));
}


RealsenseSubscriber::~RealsenseSubscriber()
{
    pAgent->Shutdown();
    pass;
}

void RealsenseSubscriber::InitializeVSLAM(std::string& configString)
{

    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_INFO(this->get_logger(), "Please provide valid voc_file and settings_file_path");
        rclcpp::shutdown();
    }

    settingsFilePath = settingsFilePath.append(configString);

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());

    sensorType = ORB_SLAM3::System::MONOCULAR;
    enablePangolinWindow = true;
    enableOpenCVWindow = true;
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl;
}

void RealsenseSubscriber::ImgCallback(const sensor_msgs::msg::Image& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
    }
    catch(cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading image");
        return;
    }
    timeStep = get_time_ns_double<sensor_msgs::msg::Image>(msg);

    // Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep);
}

std::unordered_map<std::string, std::string> get_options(int argc, char** argv)
{
    std::unordered_map<std::string, std::string> options;
    std::string previous_option = "";
    std::string current_argument = "";
    int cutoff = 0;
    for (int i = 1; i < argc; i++){
        cutoff = 0;
        current_argument = argv[i];

        if (current_argument[0] == '-') {
            if (current_argument[1] == '-' && current_argument[2] != '-') {
                cutoff = 2;
            } else if (current_argument.length() >= 3 && current_argument.substr(1,2) == "--")
            {
                std::cerr << "Invalid input format!" << std::endl;
                exit(1);
            } else {
                cutoff = 1;
            }
        
            previous_option = current_argument.substr(cutoff);
            options[previous_option] = "1"; // default symbol
        } else if (!previous_option.empty()) {
            options[previous_option] = current_argument.substr(cutoff);
            previous_option = "";
        }
    }
    return options;
}

ORB_SLAM3::System::eSensor RealsenseSubscriber::convertStringToSensorType(const std::string& str) 
{
    if (str == "MONOCULAR") {
        return ORB_SLAM3::System::eSensor::MONOCULAR;
    } else if (str == "IMU_MONOCULAR") {
        return ORB_SLAM3::System::eSensor::IMU_MONOCULAR;
    } else {
        std::cerr << "Not a valid sensor type: " << str << std::endl;
        std::cout << "Using default MONOCULAR instead..." << std::endl;
        return ORB_SLAM3::System::eSensor::MONOCULAR;
    }
}

int main(int argc, char **argv) 
{
    // if arg is MONOCULAR then ignore inertial (default)
    // if arg is IMU_MONOCULAR then include IMU
    std::unordered_map<std::string, std::string> options = get_options(argc, argv);

    rclcpp::init(argc, argv);
    
    // Create the ros node
    auto node = std::make_shared<RealsenseSubscriber>(options);
    // rclcpp::Rate rate(10); // Set the desired update rate
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}