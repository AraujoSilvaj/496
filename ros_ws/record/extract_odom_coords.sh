#!/bin/bash

# Set the path to the ROS bag file
bag_file="odom_bag_20240305_220212.bag"  # Replace with the actual bag file name

# Set the ROS topic you want to extract
topic="/odom"

# Extract information about the bag
#ros2 bag info "$bag_file" > bag_info.txt

# Extract x and y coordinates from the bag
ros2 bag play "$bag_file" --topics "$topic" | \
  rostopic echo -b "$bag_file" "$topic" #| \
  #grep -E 'pose\.pose\.position\.x|pose\.pose\.position\.y' #| \
  #awk '{print $2}' > coordinates.txt

