//
// Created by Martin Økter on 25/09/2024.
//

#ifndef ICEARM_CTRL_SRC_ICE_SERIAL_SERVO_H_
#define ICEARM_CTRL_SRC_ICE_SERIAL_SERVO_H_

#include <boost/asio.hpp>
#include <string>
#include <vector>

class IceSerialServo {
 public:
  IceSerialServo(const std::string& port, unsigned int baud_rate = 115200);
  ~IceSerialServo();

  void SendServoValues(const std::vector<int>& servo_values);

 private:
  void OpenSerialPort();

  std::string port_;
  unsigned int baud_rate_;
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;
};

#endif //ICEARM_CTRL_SRC_ICE_SERIAL_SERVO_H_
