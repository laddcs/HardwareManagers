#include <rclcpp/rclcpp.hpp>

#include <libirimager/IRDevice.h>
#include <libirimager/IRImager.h>
#include <libirimager/IRImagerClient.h>

namespace ircamera_manager
{

class IRCameraManager : public rclcpp::Node, public evo::IRImagerClient
{
    public:
        explicit IRCameraManager(const rclcpp::NodeOptions & options);
        ~IRCameraManager();

    private:
        std::shared_ptr<evo::IRDevice> dev_;
};

} // namespace ircamera_manager