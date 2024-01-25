FROM ros:humble-ros-base 
RUN apt update
RUN apt install -y apt-utils 
RUN apt install -y python3-pip
RUN pip3 install --upgrade TensorFlow
RUN apt-get install -y ros-humble-demo-nodes-cpp
RUN apt-get install -y nano
RUN apt-get install -y python3-pandas
RUN apt-get install -y python3-opencv 
RUN apt-get install python3-matplotlib
RUN apt-get install -y libgl1 
RUN apt-get install -y python3-sklearn

CMD ["bash"]

