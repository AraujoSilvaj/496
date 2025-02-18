#!/bin/bash
docker run -it --rm --net=host -v /dev/shm:/dev/shm -v /shared_local/496:/496 ros:humble-ros-base 
#docker run -it --rm --net=host -v /dev/shm:/dev/shm -v ~/496:/496 avc/devel bash -c "cd /496/ros_ws && bash"
