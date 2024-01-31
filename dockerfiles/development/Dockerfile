FROM ros:humble-ros-base   
RUN apt update
RUN apt install -y apt-utils 
RUN apt install -y python3-pip
RUN pip3 install --upgrade TensorFlow
RUN apt-get install -y ros-humble-demo-nodes-cpp \
	nano \
	vim \
	python3-pandas \
	python3-opencv  \
	python3-matplotlib \
	libgl1 \ 
	python3-sklearn


RUN rm -rf /var/lib/apt/lists/*

CMD ["bash"]

