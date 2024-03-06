#include "Y3SpaceDriver2.hpp"
#include <rclcpp/rclcpp.hpp>
#include "serial_driver/serial_driver.hpp"

const std::string Y3SpaceDriver::logger = "[ Y3SpaceDriver ] ";
const std::string Y3SpaceDriver::MODE_ABSOLUTE = "absolute";
const std::string Y3SpaceDriver::MODE_RELATIVE = "relative";

Y3SpaceDriver::Y3SpaceDriver(std::string port,
                             int baudrate,
                             int timeout,
                             std::string mode,
                             std::string frame):
    //SerialInterface(port, baudrate, timeout),
    //m_pnh(pnh),
    //m_nh(nh),
    m_mode(mode),
    m_frame(frame),
    owned_ctx{new IoContext(2)},
    serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx)},
    Node("Y3SpaceDriver")
{

    auto fc = drivers::serial_driver::FlowControl::HARDWARE;
    auto pt = drivers::serial_driver::Parity::NONE;
    auto sb = drivers::serial_driver::StopBits::ONE;
    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baudrate, fc, pt, sb);
    serial_driver_->init_port(port, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
    }
    

    //this->serialConnect();
    this->m_imuPub = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", 10);
    //this->m_tempPub = this->m_nh.advertise<std_msgs::Float64>("/imu/temp", 10);
}


Y3SpaceDriver::~Y3SpaceDriver() {}

/*

void Y3SpaceDriver::restoreFactorySettings()
{
    this->serialWriteString(RESTORE_FACTORY_SETTINGS);
}
*/

const std::string Y3SpaceDriver::getSoftwareVersion()
{
    // https://index.ros.org/p/serial_driver/
    std::vector<uint8_t>  b; //GET_FIRMWARE_VERSION_STRING
    for(int i = 0; i < strlen(GET_FIRMWARE_VERSION_STRING); i++){
        b.push_back(GET_FIRMWARE_VERSION_STRING[i]);
        printf("%c\n", GET_FIRMWARE_VERSION_STRING[i]);
    }
    serial_driver_->port()->async_send(b);
    //this->serialWriteString(GET_FIRMWARE_VERSION_STRING);

    std::vector<uint8_t>  buff;
    int s = 0;
    while(s < 2){
        s = s + serial_driver_->port()->receive(buff);
        std::string str2(buff.begin(), buff.end());
        std::cout << str2 << std::endl;
    }
    std::string str(buff.begin(), buff.end());

    //const std::string buf = this->serialReadLine();
    RCLCPP_INFO(this->get_logger(), "Software version: %s %d", str, s);
    return str;
}

/*

const std::string Y3SpaceDriver::getAxisDirection()
{
    this->serialWriteString(GET_AXIS_DIRECTION);

    const std::string buf = this->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "X: Right, Y: Up, Z: Forward";
        }
        else if ( buf == "1\r\n")
        {
            return "X: Right, Y: Forward, Z: Up";
        }
        else if ( buf == "2\r\n")
        {
            return "X: Up, Y: Right, Z: Forward";
        }
        else if (buf == "3\r\n")
        {
            return "X: Forward, Y: Right, Z: Up";
        }
        else if( buf == "4\r\n")
        {
            return "X: Up, Y: Forward, Z: Right";
        }
        else if( buf == "5\r\n")
        {
            return "X: Forward, Y: Up, Z: Right";
        }
        else if (buf == "19\r\n")
        {
            return "X: Forward, Y: Left, Z: Up";
        }
        else
        {
            ROS_WARN_STREAM(this->logger << "Buffer indicates: " + buf);
            return "Unknown";
        }
    }();

    ROS_INFO_STREAM(this->logger << "Axis Direction: " << ret);
    return ret;
}

void Y3SpaceDriver::startGyroCalibration(void)
{
    ROS_INFO_STREAM(this->logger << "Starting Auto Gyro Calibration...");
    this->serialWriteString(BEGIN_GYRO_AUTO_CALIB);
  
    rclcpp::Duration(5.0).sleep();
    ROS_INFO_STREAM(this->logger << "Proceeding");
}

void Y3SpaceDriver::setMIMode(bool on)
{
    if(on)
    {
        this->serialWriteString(SET_MI_MODE_ENABLED);
    }
    else
    {
        this->serialWriteString(SET_MI_MODE_DISABLED);
    }
}

const std::string Y3SpaceDriver::getCalibMode()
{
    this->serialWriteString(GET_CALIB_MODE);

    const std::string buf = this->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "Bias";
        }
        else if ( buf == "1\r\n")
        {
            return "Scale and Bias";
        }
        else
        {
            ROS_WARN_STREAM(this->logger << "Buffer indicates: " + buf);
            return "Unknown";
        }
    }();

    ROS_INFO_STREAM(this->logger << "Calibration Mode: " << ret);
    return ret;
}

const std::string Y3SpaceDriver::getMIMode()
{
    this->serialWriteString(GET_MI_MODE_ENABLED);

    const std::string buf = this->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "Disabled";
        }
        else if ( buf == "1\r\n")
        {
            return "Enabled";
        }
        else
        {
            ROS_WARN_STREAM(this->logger << "Buffer indicates: " + buf);
            return "Unknown";
        }
    }();

    ROS_INFO_STREAM(this->logger << "MI Mode: " << ret);
    return ret;
}

*/
//! Run the serial sync
void Y3SpaceDriver::run()
{

    printf("hello from yost\n");  // serial_driver_->port()->async_send(packet.frame())
    
    std::vector<double> parsedVals;
    sensor_msgs::msg::Imu imuMsg;
    //std_msgs::Float64 tempMsg;

    //this->startGyroCalibration();
    this->getSoftwareVersion();
    //this->getAxisDirection();
    //this->getCalibMode();
    //this->getMIMode();

    /*
    if (m_mode == MODE_ABSOLUTE)
    {
        RCLCPP_INFO(this->get_logger(),"Using absolute driver stream configuration");
        this->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_ABSOLUTE);
    }
    else if (m_mode == MODE_RELATIVE)
    {
        RCLCPP_INFO(this->get_logger(),"Using relative driver stream configuration");
        this->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_RELATIVE);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"Unknown driver mode set... Defaulting to relative");
        this->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_RELATIVE);
    }
    this->serialWriteString(TARE_WITH_CURRENT_ORIENTATION);
    this->serialWriteString(TARE_WITH_CURRENT_QUATERNION);
    this->serialWriteString(SET_STREAMING_TIMING_100_MS);
    this->serialWriteString(START_STREAMING);
    RCLCPP_INFO(this->get_logger(), "Ready\n");
  
    rclcpp::Rate rate(10);
    int line = 0;
    while(rclcpp::ok())
    {
        while(this->available() > 0)
        {
            line += 1;
            std::string buf = this->serialReadLine();
            std::string parse;
            std::stringstream ss(buf);
            double i;

            // Parse data from the line
            while (ss >> i)
            {
                parsedVals.push_back(i);
                if (ss.peek() == ',')
                ss.ignore();
            }

            // Should stop reading when line == number of tracked streams
            if(line == 4)
            {
                // Reset line tracker
                line = 0;
        
                // Prepare IMU message
                imuMsg.header.stamp           = rclcpp::Time::now();
                imuMsg.header.frame_id        = m_frame;
                imuMsg.orientation.x          = parsedVals[0];
                imuMsg.orientation.y          = parsedVals[1];
                imuMsg.orientation.z          = parsedVals[2];
                imuMsg.orientation.w          = parsedVals[3];
                imuMsg.angular_velocity.x     = parsedVals[4];
                imuMsg.angular_velocity.y     = parsedVals[5];
                imuMsg.angular_velocity.z     = parsedVals[6];
                imuMsg.linear_acceleration.x  = parsedVals[7];
                imuMsg.linear_acceleration.y  = parsedVals[8];
                imuMsg.linear_acceleration.z  = parsedVals[9];

                // Prepare temperature message        
                tempMsg.data = parsedVals[10];

                // Clear parsed values
                parsedVals.clear();

                this->m_imuPub.publish(imuMsg);
                this->m_tempPub.publish(tempMsg);
            }
        }

        // Throttle ROS at fixed Rate
        rate.sleep();
        rclcpp::spin_some(node);
    }
    */
}

