#!/usr/bin/env python3

import sys # System specific modules
import os # Operating specific functions
import glob
import time # Python timing module
import copy # For deepcopying arrays
import shutil # High level folder operation tool
from pathlib import Path # To find the "home" directory location
import argparse # To accept user arguments from commandline
import natsort # To ensure all images are chosen loaded in the correct order
import copy # For making deepcopies of openCV matrices, python lists, numpy arrays etc.
import numpy as np # Python Linear Algebra module
import cv2 # OpenCV

#* ROS2 imports
import ament_index_python.packages
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

# Import ROS2 message templates
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError # Library to convert image messages to numpy array

#* Class definition
class ReplicaDriver(Node):
    def __init__(self, node_name = "replica_py_driver"):
        super().__init__(node_name) # Initializes the rclpy.Node class. It expects the name of the node

        # Initialize parameters to be passed from the command line (or launch file)
        self.declare_parameter("replica_path", "/home/xiaoang/MyLab/replica_v1/")
        self.declare_parameter("scene_name","room_0/")

        # Parse values sent by command line
        self.replica_path = str(self.get_parameter('replica_path').value) # absolute path
        self.scene_name = str(self.get_parameter('scene_name').value)
        
        # Input lists for orbslam3 (to be published)
        self.image_list = []
        self.depth_list = []
        self.timestamp_list = []

        # DEBUG 
        self.get_logger().info(f"-------------- Received parameters --------------------------\n")
        self.get_logger().info(f"self.replica_path: {self.replica_path}")
        self.get_logger().info(f"self.scene_name: {self.scene_name}")

        # Global path definitions
        self.data_dir = os.path.join(self.replica_path, self.scene_name)
        self.image_sequence_dir = os.path.join(self.data_dir, "images/") 
        self.depth_sequence_dir = os.path.join(self.data_dir, "depths/") 
        self.timestamps_dir = os.path.join(self.data_dir, "timestamps.txt")
        
        # get the images, depth images and corresponding timestamps
        self.get_image_sequence()

        # Define a CvBridge object
        self.cv_bridge = CvBridge()
        
        self.handshake_completed = False

        # ROS2 publisher/subscriber topic names 
        self.handshake_pub_topic = "/replica_py_driver/handshake"
        self.ack_sub_topic = "/replica_subscriber_node/ack"
        self.image_pub_topic = "/replica_py_driver/image"
        self.depth_pub_topic = "/replica_py_driver/depth"
        self.timestamp_pub_topic = "/replica_py_driver/timestep"
        
        # Publisher for handshake with cpp node
        self.handshake_publisher_ = self.create_publisher(Bool, self.handshake_pub_topic, 1)
        
        # Subscriber to acknowledge that cpp node is started
        self.ack_subscriber_ = self.create_subscription(String, self.ack_sub_topic, self.ack_callback, 10)
        
        # Publisher to send RGB and depth images
        self.img_publisher_ = self.create_publisher(Image, self.image_pub_topic, 1)
        self.depth_publisher_ = self.create_publisher(Image, self.depth_pub_topic, 1)

        # Initialize work variables for main logic
        self.sequence_length = len(self.image_list)
        self.start_frame = 0
        
        self.get_logger().info(f"Sequence length: {self.sequence_length}")

        try:
            user_input = input("Enter end frame index (-1 for full sequence): ").strip()
            user_input_val = int(user_input) if user_input else -1
            self.end_frame = self.sequence_length - 1 if user_input_val == -1 else user_input_val
        except ValueError:
            self.get_logger().info("Invalid input, using full sequence.")
            self.end_frame = self.sequence_length - 1
            
        self.curr_idx = self.start_frame
        
        self.timer = self.create_timer(1.0/20.0, self.timer_callback)

        self.get_logger().info("ReplicaDriver initialized, attempting handshake with subscriber node \n")
    
    def ack_callback(self, msg):
        """
            callback for acknowledge message that subscriber node is startet
        """
        self.get_logger().info(f"Got ack: {msg.data}")
        
        if (msg.data == "ACK"):
            self.handshake_completed = True
    
    def handshake_with_cpp_node(self):
        msg = Bool()
        msg.data = True
        self.handshake_publisher_.publish(msg)
        time.sleep(0.01)
    
    def get_image_sequence(self):
        """
            Returns the image and depth image list in ascending order
        """
        self.image_list = natsort.natsorted(os.listdir(self.image_sequence_dir), reverse=False)
        self.depth_list = natsort.natsorted(os.listdir(self.depth_sequence_dir), reverse=False)
        
        with open(self.timestamps_dir, 'r') as f:
            self.timestamp_list = [int(line.strip()) for line in f if line.strip()]

    def run_sequence_publisher(self):
        """
            publish images from the benchmark sequences
        """
        # image messages
        image_msg = Image()
        depth_msg = Image()

        image_name = f"{self.curr_idx:06d}.jpg"
        depth_name = f"{self.curr_idx:06d}.png"
                
        image_path = os.path.join(self.image_sequence_dir, image_name)
        depth_path = os.path.join(self.depth_sequence_dir, depth_name)
        
        image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
                
        image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
        depth_msg = self.cv_bridge.cv2_to_imgmsg(depth, encoding="mono16")
        
        # timestamps
        timestep_ns = self.timestamp_list[self.curr_idx]
        stamp = rclpy.time.Time(seconds=timestep_ns // 10**9, nanoseconds=timestep_ns % 10**9).to_msg()
        
        image_msg.header.stamp = stamp
        depth_msg.header.stamp = stamp
        
        try:
            self.img_publisher_.publish(image_msg)
            self.depth_publisher_.publish(depth_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            
    def timer_callback(self):
        """
        timer_callback for the replica publisher node
        """
        if self.handshake_completed:
            if self.curr_idx > self.end_frame:
                self.get_logger().info("All frames published. Shutting down...")
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()
                return
        
            self.run_sequence_publisher()
            self.curr_idx += 1

# main function
def main():
    rclpy.init() # Initialize node
    node = ReplicaDriver()
    
    while (not node.handshake_completed):
        node.handshake_with_cpp_node()
        rclpy.spin_once(node)
        
        if (node.handshake_completed):
            break
    
    node.get_logger().info("------------------ORB-SLAM3 STARTS------------------")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt: shutting down manually.")
        node.destroy_node()
        rclpy.shutdown()
        

if __name__=="__main__":
    main()

