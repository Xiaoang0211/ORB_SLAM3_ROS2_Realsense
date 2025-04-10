/**
 * @file replica_subscriber.cpp
 * @author Xiaoang Zhang (jesse1008611@gmail.com)
 * @brief replica currently only supports v-slam (whether mono or rgbd)
 * @version 0.1
 * @date 2025-02-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <ros2_orb_slam3/replica/replica_subscriber.hpp>
#include <unordered_map>

ReplicaSubscriber::ReplicaSubscriber(std::unordered_map<std::string, std::string>& options) :
Node("replica_subscriber_node")
{   
    // default settings for sensors and operating mode
    ConfigFile = "Replica.yaml";
    sensorType = ORB_SLAM3::System::eSensor::RGBD;
    mode = "RGBD"; // default is rgbd

    if (!options["config"].empty()) {
        ConfigFile = (options.at("config") == "1") ? "Replica.yaml" : options["config"];  
    } else if (!options["c"].empty())  {
        ConfigFile = (options.at("c") == "1") ? "Replica.yaml" : options["c"];
    }


    if (!options["sensor_type"].empty()) {
        if (options.at("sensor_type") == "1") {
            sensorType = ORB_SLAM3::System::eSensor::RGBD;
            mode = "RGBD";
        } else {
            sensorType = convertStringToSensorType(options["sensor_type"]);
            mode = options["sensor_type"];
        }
    } else if (!options["s"].empty()) {
        if (options.at("s") == "1") {
            sensorType = ORB_SLAM3::System::eSensor::RGBD;
            mode = "RGBD";
        } else {
            sensorType = convertStringToSensorType(options["s"]);
            mode = options["s"];
        }
    }

    homeDir = getenv("HOME");
    RCLCPP_INFO(this->get_logger(), "\nReplica subscriber node started");
    replica_py_driver_activated = false;

    // this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  

    //* Watchdog, populate default values
    nodeName = "file_not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    // rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    // nodeName = param1.as_string();
    
    rclcpp::Parameter param1 = this->get_parameter("voc_file_arg");
    vocFilePath = param1.as_string();

    rclcpp::Parameter param2 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param2.as_string();

    // set config file path for realsense camera
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        vocFilePath = homeDir + "/" + packagePath + "/" + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "/" + "orb_slam3/config/" + mode + "/";
    }

    //* DEBUG print 
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settingsFilePath %s", settingsFilePath.c_str());


    // ROS2 message subscription
    // Quality of Service for sensor messages
    qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // handshake topics
    relicaDriverTopic = "/replica_py_driver/handshake";
    ackTopic = "/replica_subscriber_node/ack";

    // sensor topics
    rgbImageTopic = "/replica_py_driver/image";
    depthImageTopic = "/replica_py_driver/depth"; 

    // initialize subscriber
    replica_py_driver_sub_ = this->create_subscription<std_msgs::msg::Bool>(relicaDriverTopic, 1, std::bind(&ReplicaSubscriber::replica_driver_callback, this, std::placeholders::_1));

    // acknowledge publisher 
    ack_publisher_ = this->create_publisher<std_msgs::msg::String>(ackTopic, 10);

    // synchronized subscription for sensor data topics
    rgb_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, rgbImageTopic, qos_profile);
    depth_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, depthImageTopic, qos_profile);

    // Callback for rgb and depth images with synchronization
    image_sync_ = std::make_shared<message_filters::Synchronizer<imageSyncPolicy>>(imageSyncPolicy(10), *rgb_image_sub_,
                                                                                                        *depth_image_sub_);
                                                                                                    
    image_sync_->registerCallback(std::bind(&ReplicaSubscriber::imageCallback, this, std::placeholders::_1,
                                                                                     std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Waiting for finishing handshake ......");
}


ReplicaSubscriber::~ReplicaSubscriber()
{
    pAgent->Shutdown();
    pass;
}

void ReplicaSubscriber::replica_driver_callback(const std_msgs::msg::Bool& msg) 
{   
    replica_py_driver_activated = msg.data;

    RCLCPP_INFO(this->get_logger(), "Replica Py driver is activated");
    
    auto acknowledge = std_msgs::msg::String();
    acknowledge.data = "ACK";

    RCLCPP_INFO(this->get_logger(), "Sent response: %s", acknowledge.data.c_str());
    ack_publisher_->publish(acknowledge);

    InitializeVSLAM(ConfigFile);
}

void ReplicaSubscriber::InitializeVSLAM(std::string& configString)
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
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3: %s initialized", mode.c_str());
}


void ReplicaSubscriber::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_image_msg,
                                      const sensor_msgs::msg::Image::ConstSharedPtr& depth_image_msg)
{   
    timeStep = get_time_ns_double<sensor_msgs::msg::Image>(*rgb_image_msg);

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

    case ORB_SLAM3::System::eSensor::RGBD:
        Tcw = pAgent->TrackRGBD(rgb_ptr->image, depth_ptr->image, timeStep);
        break;
    
    default:
        RCLCPP_ERROR(this->get_logger(), "Unsupported input sensor modality!");
        return;
    }
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

ORB_SLAM3::System::eSensor ReplicaSubscriber::convertStringToSensorType(const std::string& str) 
{
    if (str == "Monocular")
        return ORB_SLAM3::System::eSensor::MONOCULAR;
    else if (str == "RGBD")
        return ORB_SLAM3::System::eSensor::RGBD;
    else {
        RCLCPP_WARN(this->get_logger(), "Not a valid sensor type: %s", str.c_str());
        RCLCPP_INFO(this->get_logger(), "Using default RGBD instead ...");
        return ORB_SLAM3::System::eSensor::RGBD;
    }
}

int main(int argc, char **argv) 
{
    std::unordered_map<std::string, std::string> options = get_options(argc, argv);

    rclcpp::init(argc, argv);
    
    // Create the ros node
    auto node = std::make_shared<ReplicaSubscriber>(options);
    // rclcpp::Rate rate(10); // Set the desired update rate
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}