#!/bin/bash


# ./496/ros_scripts/button.sh
# ./496/ros_scripts/imu_vesc_button.sh
#./496/ros_scripts/laptop-dev.sh
docker run -it --rm --net=host -d -v /dev/shm:/dev/shm -v ~/496:/496 avc/devel
docker run -it --rm --net=host -d -v /dev/shm:/dev/shm -v ~/496:/496 avc/devel

# #button container
# docker run -it --rm --net=host -d --gpus all --privileged --device=/dev/gpiochip0 \
# 														--device=/dev/gpiochip1 \
# 														--device=/dev/gpiochip2 -v /dev/shm:/dev/shm -v ~/496:/496 avc/devel

# #imu-vesc container
# docker run -it --rm --net=host -d --device=/dev/ttyVESC \
# 								--device=/dev/ttyIMU -v /dev/shm:/dev/shm -v ~/496:/496 avc/devel


containers=()

for i in $(docker ps -q); do
	containers+=($i) 
done

#echo "${containers[@]}"

# #for the one with the button 
docker exec -it -d ${containers[0]} bash -c "cd /496/ros_ws && \
source /opt/ros/humble/setup.bash && \
colcon build && \
source install/setup.bash && \
./launch_slam.sh"
#ros2 run button_publisher button_publisher"



#for the one with the launch file 
docker exec -it ${containers[1]} bash -c "cd /496/ros_ws && \
source /opt/ros/humble/setup.bash && \
colcon build && \
source install/setup.bash && \
ros2 launch launch_nodes hc_driver.launch"

