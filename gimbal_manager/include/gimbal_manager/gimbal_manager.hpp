#include <rclcpp/rclcpp.hpp>

#include <gSDK/serial_port.h>
#include <gSDK/gimbal_interface.h>

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
};

} // namespace gimbal_manager