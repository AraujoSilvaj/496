FROM osrf/ros:humble-desktop 
RUN apt-get update && apt install -y \
	ros-humble-plotjuggler-ros \
	&& rm -rf /var/lib/apt/lists/*


RUN echo -e '#!/bin/bash\nros2 run plotjuggler plotjuggler' > /usr/bin/plotjuggler && chmod +x /usr/bin/plotjuggler

CMD ["bash"]

