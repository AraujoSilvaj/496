FROM ros:humble-ros-base   
RUN apt update -y
RUN apt-get update && apt install -y \
	apt-utils \
	python3-pip \
	&& rm -rf /var/lib/apt/lists/*



RUN apt-get update && apt-get install -y  \
	nano \
	vim \
#	python3-pandas \
#	python3-opencv  \
#	python3-opencv-headless  \
#	python3-matplotlib \
#	libgl1 \ 
#	python3-sklearn \
	&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
	ros-humble-demo-nodes-cpp \
	ros-humble-serial-driver \
	ros-humble-ackermann-msgs \
	ros-humble-nmea-msgs \
	ros-humble-slam-toolbox \
	ros-humble-robot-localization \
	libqt5serialport5-dev \
	&& rm -rf /var/lib/apt/lists/*

RUN pip3 install TensorFlow pyserial opencv-python-headless

CMD ["bash"]

