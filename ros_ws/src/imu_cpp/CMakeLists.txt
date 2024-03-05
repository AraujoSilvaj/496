cmake_minimum_required(VERSION 2.8.3)
project(y3space_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
             geometry_msgs
             roscpp
             rospy
             sensor_msgs
             serial
             std_msgs)

find_package(Boost REQUIRED COMPONENTS system)

###################
## catkin Config ##
###################

include_directories(include/y3space_driver)

catkin_package(CATKIN_DEPENDS geometry_msgs
                              roscpp
                              rospy
                              sensor_msgs
                              serial
                              std_msgs)

###########
## Build ##
###########

include_directories(
    INCLUDE_DIRS include
    ${BOOST_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

add_library(serial_interface
            src/y3space_driver/SerialInterface.cpp)

add_executable(y3space_driver
               src/y3space_driver/Y3SpaceDriver.cpp
               src/y3space_driver/main.cpp)

target_link_libraries(y3space_driver
                      serial_interface
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})
