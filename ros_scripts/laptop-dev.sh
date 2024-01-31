#!/bin/bash
docker run -it --rm --net=host -v /dev/shm:/dev/shm -v ~/496:/496 ros:humble-ros-base 
