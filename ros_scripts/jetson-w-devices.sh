#!/bin/bash
docker run -it --rm --net=host --device=/dev/ttyVESC -v /dev/shm:/dev/shm -v ~/496:/496 avc/devel
docker run -it --rm --net=host --device=/dev/ttyGPS -v /dev/shm:/dev/shm -v ~/496:/496 avc/devel