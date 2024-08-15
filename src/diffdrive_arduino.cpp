#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

// Constructor
DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{
}

// Initialization callback
hardware_interface::CallbackReturn DiffDriveArduino::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Configuring...");

    time_ = std::chrono::system_clock::now();

    // Extract and store parameters from info.hardware_parameters
    cfg_.front_left_wheel_name = info.hardware_parameters.at("front_left_wheel_name");
    cfg_.front_right_wheel_name = info.hardware_parameters.at("front_right_wheel_name");
    cfg_.back_left_wheel_name = info.hardware_parameters.at("back_left_wheel_name");
    cfg_.back_right_wheel_name = info.hardware_parameters.at("back_right_wheel_name");
    cfg_.loop_rate = std::stof(info.hardware_parameters.at("loop_rate"));
    cfg_.device = info.hardware_parameters.at("device");
    cfg_.baud_rate = std::stoi(info.hardware_parameters.at("baud_rate"));
    cfg_.timeout = std::stoi(info.hardware_parameters.at("timeout"));
    cfg_.enc_counts_per_rev = std::stoi(info.hardware_parameters.at("enc_counts_per_rev"));

    // Set up the wheels
    Fl_wheel_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
    Fr_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
    Bl_wheel_.setup(cfg_.back_left_wheel_name, cfg_.enc_counts_per_rev);
    Br_wheel_.setup(cfg_.back_right_wheel_name, cfg_.enc_counts_per_rev);

    // Set up the Arduino
    arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

    RCLCPP_INFO(logger_, "Finished Configuration");

    return CallbackReturn::SUCCESS;
}

// Export state interfaces
std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
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

// Export command interfaces
std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(Fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &Fl_wheel_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(Fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &Fr_wheel_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(Bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &Bl_wheel_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(Br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &Br_wheel_.cmd));

    return command_interfaces;
}

// Activation callback
hardware_interface::CallbackReturn DiffDriveArduino::on_activate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    RCLCPP_INFO(logger_, "Starting Controller...");

    arduino_.sendEmptyMsg();
    arduino_.setPidValues(30, 20, 0, 100);

    return CallbackReturn::SUCCESS;
}

// Deactivation callback
hardware_interface::CallbackReturn DiffDriveArduino::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/)
{
    RCLCPP_INFO(logger_, "Stopping Controller...");

    return CallbackReturn::SUCCESS;
}

// Read data
hardware_interface::return_type DiffDriveArduino::read(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/)
{
    // Calculate time delta
    auto new_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_time - time_;
    double deltaSeconds = diff.count();
    time_ = new_time;

    if (!arduino_.connected())
    {
        return return_type::ERROR;
    }

    arduino_.readEncoderValues(Fl_wheel_.enc, Fr_wheel_.enc, Bl_wheel_.enc, Br_wheel_.enc);

    double pos_prev_fl = Fl_wheel_.pos;
    Fl_wheel_.pos = Fl_wheel_.calcEncAngle();
    Fl_wheel_.vel = (Fl_wheel_.pos - pos_prev_fl) / deltaSeconds;

    double pos_prev_fr = Fr_wheel_.pos;
    Fr_wheel_.pos = Fr_wheel_.calcEncAngle();
    Fr_wheel_.vel = (Fr_wheel_.pos - pos_prev_fr) / deltaSeconds;

    double pos_prev_bl = Bl_wheel_.pos;
    Bl_wheel_.pos = Bl_wheel_.calcEncAngle();
    Bl_wheel_.vel = (Bl_wheel_.pos - pos_prev_bl) / deltaSeconds;

    double pos_prev_br = Br_wheel_.pos;
    Br_wheel_.pos = Br_wheel_.calcEncAngle();
    Br_wheel_.vel = (Br_wheel_.pos - pos_prev_br) / deltaSeconds;

    return return_type::OK;
}

// Write data
hardware_interface::return_type DiffDriveArduino::write(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/)
{
    if (!arduino_.connected())
    {
        return return_type::ERROR;
    }

    arduino_.setMotorValues(
        Fl_wheel_.cmd / Fl_wheel_.rads_per_count / cfg_.loop_rate,
        Fr_wheel_.cmd / Fr_wheel_.rads_per_count / cfg_.loop_rate,
        Bl_wheel_.cmd / Bl_wheel_.rads_per_count / cfg_.loop_rate,
        Br_wheel_.cmd / Br_wheel_.rads_per_count / cfg_.loop_rate
    );

    RCLCPP_INFO(logger_, "FL Wheel Command: %f", Fl_wheel_.cmd / Fl_wheel_.rads_per_count / cfg_.loop_rate);
    RCLCPP_INFO(logger_, "FR Wheel Command: %f", Fr_wheel_.cmd / Fr_wheel_.rads_per_count / cfg_.loop_rate);
    return return_type::OK;
}

// Export the class for pluginlib
PLUGINLIB_EXPORT_CLASS(
    DiffDriveArduino,
    hardware_interface::SystemInterface
)