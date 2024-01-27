#include <gimbal_manager/gimbal_manager.hpp>

namespace gimbal_manager
{

    GimbalManager::GimbalManager(const rclcpp::NodeOptions & options) : Node("gimbal_manager", options)
    {

    }

} // namespace gimbal_interface

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gimbal_manager::GimbalManager)