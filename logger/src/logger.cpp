#include <logger/logger.hpp>

namespace logger
{
    Logger::Logger(const rclcpp::NodeOptions & options) : Node("logger", options)
    {

    }

    void energyCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {

    }

    void tempCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        
    }
} // namespace logger

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(logger::Logger)