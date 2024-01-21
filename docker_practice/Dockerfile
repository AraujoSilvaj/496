FROM ros:humble-ros-base 
RUN mkdir scripts 
RUN apt update
RUN apt-get install -y ros-humble-demo-nodes-cpp
RUN apt-get install -y python3-pandas
RUN apt-get install -y python3-opencv 
RUN python3 -m pip install tensorflow[and-cuda]
RUN apt-get install python3-numpy
RUN apt-get install python3-matplotlib
RUN apt-get install -y libgl1 
RUN apt-get install -y python3-sklearn
CMD ["bash"]
