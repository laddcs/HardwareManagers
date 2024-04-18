#include <rclcpp/rclcpp.hpp>

#include <rosbag2_cpp/writer.hpp>

#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <hardware_msgs/msg/flag.hpp>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/gimbal_device_attitude_status.hpp>

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
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermalSub_;

        rclcpp::Subscription<hardware_msgs::msg::Flag>::SharedPtr flagSub_;

        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr px4StatusSub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4VehicleOdometrySub_;
        rclcpp::Subscription<px4_msgs::msg::GimbalDeviceAttitudeStatus>::SharedPtr px4GimbalStatusSub_;

        // Subscription Callbacks
        void thermalCB(std::shared_ptr<rclcpp::SerializedMessage> msg) const;

        void flagCB(std::shared_ptr<rclcpp::SerializedMessage> msg) const;

        void px4StatusCB(const px4_msgs::msg::VehicleStatus::ConstSharedPtr msg);
        void px4VehicleOdometryCB(std::shared_ptr<rclcpp::SerializedMessage> msg) const;
        void px4GimbalStatusCB(std::shared_ptr<rclcpp::SerializedMessage> msg) const;

        void initializeParameters();

        void openBag();

};

} //namespace logger