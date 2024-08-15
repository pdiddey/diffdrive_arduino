#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include "diffdrive_arduino/diffdrive_arduino.h"


int main(int argc, char **argv)
{
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("diffdrive_robot");

  // Set up the DiffDriveArduino configuration from parameters
  DiffDriveArduino::Config robot_cfg;
  node->declare_parameter<std::string>("front_left_wheel_name", "fl_wheel");
  node->declare_parameter<std::string>("front_right_wheel_name", "fr_wheel");
  node->declare_parameter<std::string>("back_left_wheel_name", "bl_wheel");
  node->declare_parameter<std::string>("back_right_wheel_name", "br_wheel");
  node->declare_parameter<int>("baud_rate", 57600);
  node->declare_parameter<std::string>("device", "/dev/ttyACM0");
  node->declare_parameter<int>("enc_counts_per_rev", 374);
  node->declare_parameter<double>("robot_loop_rate", 10.0);

  node->get_parameter("front_left_wheel_name", robot_cfg.front_left_wheel_name);
  node->get_parameter("front_right_wheel_name", robot_cfg.front_right_wheel_name);
  node->get_parameter("back_left_wheel_name", robot_cfg.back_left_wheel_name);
  node->get_parameter("back_right_wheel_name", robot_cfg.back_right_wheel_name);
  node->get_parameter("baud_rate", robot_cfg.baud_rate);
  node->get_parameter("device", robot_cfg.device);
  node->get_parameter("enc_counts_per_rev", robot_cfg.enc_counts_per_rev);
  node->get_parameter("robot_loop_rate", robot_cfg.loop_rate);

  // Initialize the robot hardware interface
  auto robot = std::make_shared<DiffDriveArduino>(robot_cfg);
  auto cm = std::make_shared<controller_manager::ControllerManager>(robot, node);

  rclcpp::Rate loop_rate(robot_cfg.loop_rate);
  rclcpp::Time prev_time = node->now();

  while (rclcpp::ok())
  {
    robot->read();
    cm->update(node->now(), node->now() - prev_time);
    prev_time = node->now();
    robot->write();

    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}