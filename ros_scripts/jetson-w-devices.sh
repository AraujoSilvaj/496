#!/bin/bash

devices=("ttyVESC" "ttyGPS" "ttyIMU")

for device in "${devices[@]}"; do
    docker run -it --rm --net=host --device=/dev/$device -v /dev/shm:/dev/shm -v ~/496:/496 avc/devel
done
