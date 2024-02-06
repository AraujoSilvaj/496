#!/bin/bash
docker run -it --rm --net=host --gpus all --device=/dev/video0 -v /dev/shm:/dev/shm -v ~/496:/496 ros:humble-ros-base 
