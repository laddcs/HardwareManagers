#include <rclcpp/rclcpp.hpp>

#include <gSDK/serial_port.h>
#include <gSDK/gimbal_interface.h>
#include <gSDK/gimbal_protocol.h>

#include <geometry_msgs/msg/quaternion_stamped.hpp>



namespace gimbal_manager
{

class GimbalManager : public rclcpp::Node
{
    public:
        explicit GimbalManager(const rclcpp::NodeOptions & options);
        ~GimbalManager();

    private:
        std::shared_ptr<Serial_Port> serial_port_;
        std::shared_ptr<Gimbal_Interface> gimbal_interface_;

        void initializeParameters();
        void attitude_timer_callback();

        rclcpp::TimerBase::SharedPtr attitude_timer_;
        rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_publisher_;
};

} // namespace gimbal_manager