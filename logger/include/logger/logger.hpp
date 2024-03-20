#include <rclcpp/rclcpp.hpp>

#include <rosbag2_cpp/writer.hpp>

#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <hardware_msgs/msg/flag.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

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

        // State
        bool bagOpen_;
        int bagNum_;

        // Subscriptions
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr energySub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermalSub_;
        rclcpp::Subscription<hardware_msgs::msg::Flag>::SharedPtr flagSub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr px4StatusSub_;

        // Subscription Callbacks
        void energyCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const;
        void thermalCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const;
        void flagCB(const hardware_msgs::msg::Flag::ConstSharedPtr & msg) const;
        void px4StatusCB(const px4_msgs::msg::VehicleStatus::ConstPtr msg);

        void initializeParameters();

        void openBag();

};

} //namespace logger