#!/bin/bash
docker run -it --rm --net=host --gpus all --device=/dev/video0 -v /dev/shm:/dev/shm -v ~/496:/496 7806f6d82dac 
#ros:humble-ros-base 
