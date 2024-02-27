#include <SerialInterface.h>

SerialInterface::SerialInterface(std::string port, int baudrate, int timeout):
	m_logger("[ Y3SSerialInterface ] ")
{
    m_port = port;
    m_baudrate = baudrate;
    m_timeout = timeout;
}

SerialInterface::~SerialInterface()
{
    if (m_connection != NULL)
    {
      	if(m_connection->isOpen())
      	{
        	ROS_INFO_STREAM(this->m_logger << " Closing the Serial Port");
        	m_connection->close();
      	}
    }
}

void SerialInterface::serialConnect()
{
	try
	{
  		m_connection.reset(new Serial(m_port, (uint32_t)m_baudrate, Timeout::simpleTimeout(m_timeout)));
	}
	catch(IOException &e)
	{
  		std::string ioerror = e.what();
  		ROS_ERROR_STREAM(this->m_logger << "Unable to connect port: " << m_port.c_str());
  		ROS_ERROR_STREAM(this->m_logger << "Is the device pluggged in? Is the serial port open?\nError: "  << ioerror.c_str());
	}

	if(m_connection && m_connection->isOpen())
	{
  		ROS_INFO_STREAM(this->m_logger
                  		<< "Connection Established with Port: " << m_port.c_str()
                  		<< " with baudrate: " << m_baudrate);
	}
}

void SerialInterface::serialWrite(uint8_t *buf, size_t len)
{
    size_t written = this->m_connection->write(buf, len);
    if (written != len)
    {
    	ROS_WARN_STREAM(this->m_logger << "Len: " << len  << "; Written: " << written);
    }
}

void SerialInterface::serialWriteString(const std::string& str)
{
    size_t written = this->m_connection->write(str);
}

uint8_t SerialInterface::serialReadByte()
{
    uint8_t buf;
    if(this->m_connection->available() > 0)
    {
      	size_t bytes = this->m_connection->read(&buf, 1);
      	if (bytes != 1)
      	{
        	ROS_WARN_STREAM(this->m_logger << "Unable to read");
      	}
    }
    return buf;
}

std::string SerialInterface::serialReadLine()
{
    std::string str = this->m_connection->readline();
    return str;
}

uint8_t* SerialInterface::serialReadBytes(size_t nbytes)
{
    uint8_t *buf = (uint8_t *)malloc(sizeof(uint8_t) * nbytes);
    if(this->m_connection->available() > 0)
    {
      	size_t bytes = this->m_connection->read(buf, nbytes);
      	if (bytes != nbytes)
      	{
        	ROS_WARN_STREAM(this->m_logger << "Unable to read");
      	}
    }
    return buf;
}

size_t SerialInterface::available()
{
    return this->m_connection->available();
}
