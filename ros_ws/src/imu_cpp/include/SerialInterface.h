#ifndef _Y3Space_SERIAL_INTERFACE_H
#define _Y3Space_SERIAL_INTERFACE_H

#include <malloc.h>
#include <ros/ros.h>
#include <serial/serial.h>

using namespace serial;


class SerialInterface
{

private:
    using SerialPtr = std::unique_ptr<serial::Serial>;

public:
    //!
    //! Constructor
    //!  
    SerialInterface(std::string port, int baudrate, int timeout);
    //!
    //! Destructor
    //!
    virtual ~SerialInterface();
    //!
    //! Establish serial connection
    //!
    virtual void serialConnect();
    //!
    //! Write to byte serial
    //!
    virtual void serialWrite(uint8_t *buf, size_t len);
    //!
    //!  Write string to serial
    //!
    virtual void serialWriteString(const std::string& str);
    //!
    //! Read byte from serial
    //!
    virtual uint8_t serialReadByte();
    //!
    //! Read line from serial
    //!
    virtual std::string serialReadLine();
    //!
    //! Read variable number of bytes from serial
    //!
    virtual uint8_t* serialReadBytes(size_t nbytes);
    //!
    //! Check if port is available and has bytes
    //!
    virtual size_t available();
    //!
    //! Getter for baudrate
    //!
    const int& getBaudRate() { return m_baudrate; }
    //!
    //! Getter for serial port
    //!
    const std::string& getSerialPort() { return m_port; }

private:
    //!
    //! name of the devices port
    //!
    std::string m_port;
    //!
    //! baudrate
    //!
    int m_baudrate;
    //!
    //! timeout in millis
    //!
    int m_timeout;
    //!
    //! connection related variables
    //!
    SerialPtr m_connection;
    //!
    //! logger zone
    //!
    const std::string m_logger;
};
#endif //_Y3Space_SERIAL_INTERFACE_H
