/**
 * @file realsense_subscriber.cpp
 * @author Xiaoang Zhang (jesse1008611@gmail.com)
 * @brief we use message synchronizer to ensure that the camera and IMU data are synchronized
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
    mode = "Monocular";

    if (!options["config"].empty()) {
        realsenseConfigFile = (options.at("config") == "1") ? "RealSense_D435i.yaml" : options["config"];  
    } else if (!options["c"].empty())  {
        realsenseConfigFile = (options.at("c") == "1") ? "RealSense_D435i.yaml" : options["c"];
    }


    if (!options["sensor_type"].empty()) {
        if (options.at("sensor_type") == "1") {
            sensorType = ORB_SLAM3::System::eSensor::MONOCULAR;
            mode = "Monocular";
        } else {
            sensorType = convertStringToSensorType(options["sensor_type"]);
            mode = options["sensor_type"];
        }
    } else if (!options["s"].empty()) {
         if (options.at("s") == "1") {
            sensorType = ORB_SLAM3::System::eSensor::MONOCULAR;
            mode = "Monocular";
        } else {
            sensorType = convertStringToSensorType(options["s"]);
            mode = options["s"];
        }
    }

    homeDir = getenv("HOME");
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  

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
        vocFilePath = homeDir + "/" + packagePath + "/" + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        // TODO: add more modes than just monocular
        settingsFilePath = homeDir + "/" + packagePath + "/" + "orb_slam3/config/" + mode + "/";
    }

    InitializeVSLAM(realsenseConfigFile);

    //* DEBUG print 
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

    // ROS2 message subscription
    qos_profile = rmw_qos_profile_sensor_data;

    // Realsense sensor topic names
    rgbImageTopic = "/camera/camera/color/image_raw";
    depthImageTopic = "/camera/camera/depth/image_raw"; 
    accelTopic = "/camera/camera/accel/sample"; 
    gyroTopic = "/camera/camera/gyro/sample";

    // initialize subscribers
    accel_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, accelTopic, qos_profile);
    gyro_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, gyroTopic, qos_profile);
    rgb_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, rgbImageTopic, qos_profile);
    depth_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, depthImageTopic, qos_profile);

    // Callbacks with synchronization
    imu_sync_ = std::make_shared<message_filters::Synchronizer<imuSyncPolicy>>(imuSyncPolicy(10), *accel_sub_, *gyro_sub_);
    image_sync_ = std::make_shared<message_filters::Synchronizer<imageSyncPolicy>>(imageSyncPolicy(10), *rgb_image_sub_, *depth_image_sub_);
    
    imu_sync_->registerCallback(std::bind(&RealsenseSubscriber::imuCallback, this, std::placeholders::_1,
                                                                                   std::placeholders::_2));

    image_sync_->registerCallback(std::bind(&RealsenseSubscriber::imageCallback, this, std::placeholders::_1,
                                                                                       std::placeholders::_2));
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

    enablePangolinWindow = true;
    enableOpenCVWindow = true;
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << mode.c_str() << " mode node initialized" << std::endl;
}

// void RealsenseSubscriber::ImgCallback(const sensor_msgs::msg::Image& msg)
// {   
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
//     }
//     catch(cv_bridge::Exception& e)
//     {
//         RCLCPP_ERROR(this->get_logger(), "Error reading image");
//         return;
//     }
//     timeStep = get_time_ns_double<sensor_msgs::msg::Image>(msg);

//     // Perform all ORB-SLAM3 operations in Monocular mode
//     //! Pose with respect to the camera coordinate frame not the world frame
//     Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep);
// }

void RealsenseSubscriber::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& accel_msg,
                                      const sensor_msgs::msg::Imu::ConstSharedPtr& gyro_msg)
{   
    std::lock_guard<std::mutex> lock(imu_mutex_);

    double imuTime = get_time_ns_double<sensor_msgs::msg::Imu>(*accel_msg);

    // convert the imu data to the orb-slam3 customized format
    cv::Point3f accel_meas = cv::Point3f(static_cast<float>(accel_msg->linear_acceleration.x),
                                         static_cast<float>(accel_msg->linear_acceleration.y),
                                         static_cast<float>(accel_msg->linear_acceleration.z));
    cv::Point3f gyro_meas = cv::Point3f(static_cast<float>(gyro_msg->angular_velocity.x),
                                         static_cast<float>(gyro_msg->angular_velocity.y),
                                         static_cast<float>(gyro_msg->angular_velocity.z));

    imu_buffer_.push_back(ORB_SLAM3::IMU::Point(accel_meas, gyro_meas, imuTime));

    while (!imu_buffer_.empty() && imu_buffer_.front().t < imuTime - 1.0) {
        imu_buffer_.erase(imu_buffer_.begin());
    }
}

void RealsenseSubscriber::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_image_msg,
                                        const sensor_msgs::msg::Image::ConstSharedPtr& depth_image_msg)
{   
    // we use the timestamp of the rgb image as input to rob slam3
    timeStep = get_time_ns_double<sensor_msgs::msg::Image>(*rgb_image_msg);

    std::vector<ORB_SLAM3::IMU::Point> imu_meas;

    // get imu data from the last to the current frame
    {
        std::lock_guard<std::mutex> lock(imu_mutex_); // mutex within this scope
        
        auto iterator = imu_buffer_.begin();
        while (iterator != imu_buffer_.end() && iterator->t < timeStep) {
            if (iterator->t >= lastTimeStep) {
                imu_meas.push_back(*iterator);
            }
            ++iterator;
        }
        // remove the old imu data that will be no more needed
        imu_buffer_.erase(imu_buffer_.begin(), iterator);
    }

    // rgb and depth image data
    cv_bridge::CvImagePtr rgb_ptr, depth_ptr;
    try
    {
        rgb_ptr = cv_bridge::toCvCopy(rgb_image_msg); // Local scope
        depth_ptr = cv_bridge::toCvCopy(depth_image_msg); // Local scope
    }
    catch(cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading image");
        return;
    }

    Sophus::SE3f Tcw;

    switch (sensorType)
    {
    case ORB_SLAM3::System::eSensor::MONOCULAR:
        Tcw = pAgent->TrackMonocular(rgb_ptr->image, timeStep);
        break;

    case ORB_SLAM3::System::eSensor::IMU_MONOCULAR:
        Tcw = pAgent->TrackMonocular(rgb_ptr->image, timeStep, imu_meas);
        break;

    case ORB_SLAM3::System::eSensor::RGBD:
        Tcw = pAgent->TrackRGBD(rgb_ptr->image, depth_ptr->image, timeStep);
        break;
    
    case ORB_SLAM3::System::eSensor::IMU_RGBD:
        Tcw = pAgent->TrackRGBD(rgb_ptr->image, depth_ptr->image, timeStep, imu_meas);
        break;

    default:
        RCLCPP_ERROR(this->get_logger(), "Unsupported input sensor modality!");
        return;
    }

    lastTimeStep = timeStep;
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
    if (str == "Monocular")
        return ORB_SLAM3::System::eSensor::MONOCULAR;
    else if (str == "Monocular-Inertial")
        return ORB_SLAM3::System::eSensor::IMU_MONOCULAR;
    else if (str == "RGBD")
        return ORB_SLAM3::System::eSensor::RGBD;
    else if (str == "RGBD-Inertial")
        return ORB_SLAM3::System::eSensor::IMU_RGBD;
    else {
        RCLCPP_WARN(this->get_logger(), "Not a valid sensor type: %s", str.c_str());
        RCLCPP_INFO(this->get_logger(), "Using default MONOCULAR instead ...");
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