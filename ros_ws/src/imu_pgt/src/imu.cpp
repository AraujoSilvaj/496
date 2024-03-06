#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include "Y3SpaceDriver2.hpp"
// /opt/ros/humble/include/std_msgs/std_msgs/msg/string.hpp
// /opt/ros/humble/include/sensor_msgs/sensor_msgs/msg/imu.hpp

class IMU_Node : public rclcpp::Node
{
public:
  IMU_Node()
  : Node("IMU_node")
  {
    this->declare_parameter("port", "/dev/ttyIMU");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("timeout", 60000);
    this->declare_parameter("mode", "relative");
    this->declare_parameter("frame", "imu_link");

    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    int timeout = this->get_parameter("timeout").as_int();
    std::string mode = this->get_parameter("mode").as_string();
    std::string frame = this->get_parameter("frame").as_string();


    printf("hello world imu_pgt package\n");

    Y3SpaceDriver driver(port, baudrate, timeout, mode, frame);
    driver.run();
  }
/*
  void timer_callback()
  {
    std::string my_param = this->get_parameter("my_parameter").as_string();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
  }
  */

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMU_Node>());
  rclcpp::shutdown();
  return 0;
}

/*
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world imu_pgt package\n");
  return 0;
}
*/
