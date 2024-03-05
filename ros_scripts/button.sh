#!/bin/bash
docker run -it --rm --net=host --gpus all --privileged --device=/dev/gpiochip0 --device=/dev/gpiochip1 --device=/dev/gpiochip2 -v /dev/shm:/dev/shm -v ~/496:/496 avc/devel

