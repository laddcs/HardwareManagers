#include <streamer/streamer.hpp>

namespace streamer
{
    Streamer::Streamer(const rclcpp::NodeOptions & options) : Node("streamer", options)
    {
        
    }
} // namespace streamer'

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(streamer::Streamer)