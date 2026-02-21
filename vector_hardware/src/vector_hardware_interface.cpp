#include "vector_hardware_interface/vector_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace vector_hardware
{
    hardware_interface::CallbackReturn VectorHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
    {
        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;
        serial_port_ = info_.hardware_parameters.at("serial_port");
        baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
        for (const auto & joint : info_.joints)
        {
            servo_ids_.push_back(std::stoi(joint.parameters.at("servo_id")));
            offsets_.push_back(std::stoi(joint.parameters.at("offset")));
            directions_.push_back(std::stoi(joint.parameters.at("direction")));
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VectorHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "Opening serial port %s at %d baud", serial_port_.c_str(), baudrate_);
        const bool open_status = sms_sts_.begin(baudrate_, serial_port_.c_str());
        if (open_status)
        {
            for (const auto & id : servo_ids_)
            {
                if (sms_sts_.Ping(id) == -1)
                {
                    RCLCPP_ERROR(this->get_logger(), "Servo ID %d not responding", id);
                    return hardware_interface::CallbackReturn::ERROR;
                }
                RCLCPP_INFO(this->get_logger(), "Servo ID %d found", id);
            }
            return hardware_interface::CallbackReturn::SUCCESS;
        }
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", serial_port_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    hardware_interface::CallbackReturn VectorHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        for (size_t i = 0; i < servo_ids_.size(); ++i)
        {
            const int id = servo_ids_.at(i);
            sms_sts_.EnableTorque(id, 1);
            const int position_steps = sms_sts_.ReadPos(id);
            const int velocity_steps_per_sec = sms_sts_.ReadSpeed(id);
            const int direction = directions_.at(i);
            const int offset = offsets_.at(i);
            const double position_rad =  direction * (position_steps - offset) * ((2.0 * M_PI) / 4096.0);
            const double velocity_rad_per_sec = direction * velocity_steps_per_sec * ((2.0 * M_PI) / 4096.0);
            const std::string name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            const std::string name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
            set_state(name_pos, position_rad);
            set_state(name_vel, velocity_rad_per_sec);
            set_command(name_pos, position_rad);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VectorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        for (const auto & id : servo_ids_)
        {
            sms_sts_.EnableTorque(id, 0);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VectorHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        sms_sts_.end();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type VectorHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        (void)period;
        for (size_t i = 0; i < servo_ids_.size(); ++i)
        {
            const int id = servo_ids_.at(i);
            const int position_steps = sms_sts_.ReadPos(id);
            if (position_steps == -1)
                continue;
            const int velocity_steps_per_sec = sms_sts_.ReadSpeed(id);
            const int direction = directions_.at(i);
            const int offset = offsets_.at(i);
            const double position_rad = direction * (position_steps - offset) * ((2.0 * M_PI) / 4096.0);
            const double velocity_rad_per_sec = direction * velocity_steps_per_sec * ((2.0 * M_PI) / 4096.0);
            const std::string name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            const std::string name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
            set_state(name_pos, position_rad);
            set_state(name_vel, velocity_rad_per_sec);
        }
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type VectorHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        (void)period;
        for (size_t i = 0; i < servo_ids_.size(); ++i)
        {
            const int id = servo_ids_.at(i);
            const int direction = directions_.at(i);
            const int offset = offsets_.at(i);
            const std::string name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            const double position_rad = get_command(name_pos);
            const int position_steps = static_cast<int>((position_rad * direction) * (4096.0 / (2.0 * M_PI))) + offset;
            sms_sts_.WritePosEx(id, position_steps, 100, 50);
        }
        return hardware_interface::return_type::OK;
    }
} // namespace vector_hardware

PLUGINLIB_EXPORT_CLASS(
    vector_hardware::VectorHardwareInterface,
    hardware_interface::SystemInterface
)