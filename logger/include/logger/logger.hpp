#include <rclcpp/rclcpp.hpp>

#include <rosbag2_cpp/writer.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace logger
{

class Logger : public rclcpp::Node
{
    public:
        Logger(const rclcpp::NodeOptions & options);

    private:
        // Bag writer object
        std::unique_ptr<rosbag2_cpp::Writer> writer_;

        // Subscriptions
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr energySub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr tempSub_;

        // Subscription Callbacks
        void energyCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
        void tempCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
};

} //namespace logger