#include "diffdrive_arduino/fake_robot.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"


FakeRobot::FakeRobot()
  : logger_(rclcpp::get_logger("FakeRobot"))
{}

hardware_interface::CallbackReturn FakeRobot::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.back_left_wheel_name = info_.hardware_parameters["back_left_wheel_name"];
  cfg_.back_right_wheel_name = info_.hardware_parameters["back_right_wheel_name"];

  // Set up the wheels
  // Note: It doesn't matter that we haven't set encoder counts per rev
  // since the fake robot bypasses the encoder code completely

  Fl_wheel_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
  Fr_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  Bl_wheel_.setup(cfg_.back_left_wheel_name, cfg_.enc_counts_per_rev);
  Br_wheel_.setup(cfg_.back_right_wheel_name, cfg_.enc_counts_per_rev);

  RCLCPP_INFO(logger_, "Finished Configuration");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FakeRobot::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(Fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &Fl_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(Fl_wheel_.name, hardware_interface::HW_IF_POSITION, &Fl_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(Fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &Fr_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(Fr_wheel_.name, hardware_interface::HW_IF_POSITION, &Fr_wheel_.pos));

  state_interfaces.emplace_back(hardware_interface::StateInterface(Bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &Bl_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(Bl_wheel_.name, hardware_interface::HW_IF_POSITION, &Bl_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(Br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &Br_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(Br_wheel_.name, hardware_interface::HW_IF_POSITION, &Br_wheel_.pos));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FakeRobot::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(Fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &Fl_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(Fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &Fr_wheel_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(Bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &Bl_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(Br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &Br_wheel_.cmd));
  return command_interfaces;
}


hardware_interface::CallbackReturn FakeRobot::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeRobot::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Stopping Controller...");

  return CallbackReturn::SUCCESS;;
}

hardware_interface::return_type FakeRobot::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // TODO fix chrono duration

  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;


  // Force the wheel position
  Fl_wheel_.pos = Fl_wheel_.pos + Fl_wheel_.vel * deltaSeconds;
  Fr_wheel_.pos = Fr_wheel_.pos + Fr_wheel_.vel * deltaSeconds;

  Bl_wheel_.pos = Bl_wheel_.pos + Bl_wheel_.vel * deltaSeconds;
  Br_wheel_.pos = Br_wheel_.pos + Br_wheel_.vel * deltaSeconds;

  return return_type::OK;

  
}

hardware_interface::return_type FakeRobot::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // Set the wheel velocities to directly match what is commanded

  Fl_wheel_.vel = Fl_wheel_.cmd;
  Fr_wheel_.vel = Fr_wheel_.cmd;

  Bl_wheel_.vel = Bl_wheel_.cmd;
  Br_wheel_.vel = Br_wheel_.cmd;

  return return_type::OK;  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  FakeRobot,
  hardware_interface::SystemInterface
)