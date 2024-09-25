//
// Created by Martin Økter on 25/09/2024.
//

#include "../include/icearm_ctrl/ice_serial_servo.h"

IceSerialServo::IceSerialServo(const std::string &port, unsigned int baud_rate)
: port_(port)
, baud_rate_(baud_rate)
, io_service_()
, serial_port_(io_service_)
{
  OpenSerialPort();
}

IceSerialServo::~IceSerialServo() {
  if (serial_port_.is_open()) {
    serial_port_.close();
  }
}

void IceSerialServo::OpenSerialPort() {
  try {
    serial_port_.open(port_);
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));

    // Wait for the connection to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout << "Connected to TinyPICO on " << port_ << " at " << baud_rate_
              << " baud." << std::endl;
  } catch (const boost::system::system_error& e) {
    std::cerr << "Error opening serial port: " << e.what() << std::endl;
    throw;
  }
}

void IceSerialServo::SendServoValues(const std::vector<int> &servo_values) {
  if (servo_values.size() != 6) {
    throw std::invalid_argument("You must provide exactly 6 servo values.")
  }

  if (!serial_port_.is_open()) {
    throw std::runtime_error("Serial port is not open.")
  }

  // Convert servo values to a comma-separated string
  std::ostringstream oss;
  for (size_t i = 0; i < servo_values.size(); ++i) {
    oss << servo_values[i];
    if (i < servo_values.size() - 1) {
      oss << ',';
    }
  }
  oss << '\n';  // Add newline character

  std::string servo_data = oss.str();

  //Write the data to the serial port
  try {
    boost::asio::write(serial_port_, boost::asio::buffer(servo_data));
    std::cout << "Sent servo data: " << servo_data;
  } catch (const boost::system::system_error& e) {
    std::cerr << "Error writing to serial port: " << e.what() << std::endl;
    throw;
  }
}