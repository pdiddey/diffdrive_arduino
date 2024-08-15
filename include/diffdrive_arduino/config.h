#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>


struct Config
{
  std::string front_left_wheel_name = "front_left_wheel";
  std::string front_right_wheel_name = "front_right_wheel";
  std::string back_left_wheel_name = "back_left_wheel";
  std::string back_right_wheel_name = "back_right_wheel";
  float loop_rate = 30;
  std::string device = "/dev/ttyACM0";
  int baud_rate = 57600;
  int timeout = 1000;
  int enc_counts_per_rev = 374;
};


#endif // DIFFDRIVE_ARDUINO_CONFIG_H