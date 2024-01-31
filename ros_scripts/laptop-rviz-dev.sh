#!/bin/bash
# Uses osrf/ros:humble-desktop which contains all the GUI dev tools

#docker run -it --rm --net=host -v /dev/shm:/dev/shm -v ~/496:/496 ros:humble-ros-base 

xhost +
docker run -it --rm --net=host -v /dev/shm:/dev/shm \
	-v ~/496:/496  \
	-v "$HOME/.Xauthority:/root/.Xauthority:rw" \
	-e DISPLAY=$DISPLAY  \
	osrf/ros:humble-desktop
