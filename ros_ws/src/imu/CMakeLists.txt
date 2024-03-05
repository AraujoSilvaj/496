cmake_minimum_required(VERSION 3.8)
project(y3space_driver)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system filesystem thread)

ament_include_dependencies(
    include/y3space_driver
)


###########
## BUILD ##
###########


ament_include_dependencies(
    INCLUDE_DIRS include 
    ${Boost_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/y3space_driver/SerialInterface.cpp
)

target_link_libraries(target ${Boost_LIBRARIES})

ament_target_dependencies(
    y3space_driver
    serial_interface
    rclcpp
    std_msgs
    geometry_msgs
    serial_driver
)

ament_auto_ad_executables(
    y3space_driver 
    src/y3space_driver/Y3SpaceDriver.cpp
    src/y3space_driver/main.cpp
)

#############
## INSTALL ##
#############

install(
    TARGETS y3space_driver 
    DESTINATION Lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include 
)
ament_export_include_directories(inlcude)

#############
## TESTING ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

ament_package()
