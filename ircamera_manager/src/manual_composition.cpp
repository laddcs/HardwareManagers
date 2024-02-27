#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <ircamera_manager/ircamera_manager.hpp>

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    auto node = std::make_shared<ircamera_manager::IRCameraManager>(options);

    exec.add_node(node);

    exec.spin();

    rclcpp::shutdown();

}