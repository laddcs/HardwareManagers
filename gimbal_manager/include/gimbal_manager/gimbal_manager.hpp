#include <rclcpp/rclcpp.hpp>

#include <gSDK/serial_port.h>
#include <gSDK/gimbal_interface.h>

namespace gimbal_manager
{

class GimbalManager : public rclcpp::Node
{
    public:
        explicit GimbalManager(const rclcpp::NodeOptions & options);

    private:
        Serial_Port *serial_port_;
        Gimbal_Interface *gimbal_interface_;
};

} // namespace gimbal_manager