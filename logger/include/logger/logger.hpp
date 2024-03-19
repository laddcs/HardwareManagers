#include <rclcpp/rclcpp.hpp>

#include <rosbag2_cpp/writer.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <hardware_msgs/msg/flag.hpp>

namespace logger
{

class Logger : public rclcpp::Node
{
    public:
        Logger(const rclcpp::NodeOptions & options);
        ~Logger();

    private:
        // Bag writer object
        std::unique_ptr<rosbag2_cpp::Writer> writer_;

        // Subscriptions
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr energySub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermalSub_;
        rclcpp::Subscription<hardware_msgs::msg::Flag>::SharedPtr flagSub_;

        // Subscription Callbacks
        void energyCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
        void thermalCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
        void flagCB(const hardware_msgs::msg::Flag::ConstSharedPtr & msg);

        void initializeParameters();

        void createBag();
};

} //namespace logger