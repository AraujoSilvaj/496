FROM ros:humble-ros-base 
RUN mkdir scripts 
RUN apt update
RUN apt install -y python3-pip
RUN apt-get install -y apt-utils
RUN apt-get install -y nano
RUN apt-get install -y ros-humble-demo-nodes-cpp
RUN apt-get install -y python3-pandas
RUN apt-get install -y python3-opencv 
RUN python3 -m pip3 install tensorflow
RUN apt-get install python3-numpy
RUN apt-get install python3-matplotlib
RUN apt-get install -y libgl1 
RUN apt-get install -y python3-sklearn
CMD ["bash"]
