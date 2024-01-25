#!/bin/bash
docker run -it --rm --net=host --device=/dev/video0 -v /dev/shm:/dev/shm -v ~/496:/496 --gpus all nvidia/cuda:11.0-base nvidia-smi  ros:humble-ros-base 
