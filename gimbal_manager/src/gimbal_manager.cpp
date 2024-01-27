#include <gimbal_manager/gimbal_manager.hpp>

namespace gimbal_manager
{

    GimbalManager::GimbalManager(const rclcpp::NodeOptions & options) : Node("gimbal_manager", options)
    {
        // Initialize Parameters
        initializeParameters();

        // Get serial port parameters
        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();

        RCLCPP_INFO(this->get_logger(), "Opening Port: %s, With Baud Rate: %i", port.c_str(), baud);

        // Create the port manager
        serial_port_ = std::make_shared<Serial_Port>((char *)port.c_str(), baud);

        // Get interface parameteres
        int protocol = this->get_parameter("gimbal_protocol").as_int();

        RCLCPP_INFO(this->get_logger(), "Using Protocol: %i", protocol);

        // Initialize the interface
        if(protocol == 0)
        {
            gimbal_interface_ = std::make_shared<Gimbal_Interface>(
                serial_port_.get(), 1, 
                MAV_COMP_ID_ONBOARD_COMPUTER,
                Gimbal_Interface::MAVLINK_GIMBAL_V1
            );
        } else if (protocol == 1)
        {
            gimbal_interface_ = std::make_shared<Gimbal_Interface>(
                serial_port_.get(), 1, 
                MAV_COMP_ID_ONBOARD_COMPUTER,
                Gimbal_Interface::MAVLINK_GIMBAL_V2
            );   
        } else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid Protocol! Fail to start!");
            return;
        }

        // Start the serial port and the interface
        RCLCPP_INFO(this->get_logger(), "Opening Port");
        serial_port_->start();

        RCLCPP_INFO(this->get_logger(), "Starting Interface");
        gimbal_interface_->start();
    }

    GimbalManager::~GimbalManager()
    {
        // Explicitky call serial port and interface destructors
        try
        {
            gimbal_interface_->handle_quit(0);
        }
        catch(int error){}

        try
        {
            serial_port_->handle_quit(0);
        }
        catch(int error){}
    }

    void GimbalManager::initializeParameters()
    {
        this->declare_parameter("serial_port", rclcpp::ParameterValue("/dev/ttyUSB0"));
        this->declare_parameter("baud_rate", rclcpp::ParameterValue(115200));
        this->declare_parameter("gimbal_protocol", rclcpp::ParameterValue(0));
    }

} // namespace gimbal_interface

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gimbal_manager::GimbalManager)