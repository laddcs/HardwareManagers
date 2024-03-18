#include <logger/logger.hpp>
#include <filesystem>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace logger
{
    Logger::Logger(const rclcpp::NodeOptions & options) : Node("logger", options)
    {
        // Initialize Parameters
        initializeParameters();

        // Set QoS profile for node
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);


        writer_ = std::make_unique<rosbag2_cpp::Writer>();

        std::string filePath = this->get_parameter("log_path").as_string();
        std::string filePrefix = this->get_parameter("log_prefix").as_string();
        std::string logFile = filePath + filePrefix;
        
        writer_->open(logFile);

        writer_->create_topic(
            {
                "thermal_image",
                "sensor_msgs/msg/Image",
                rmw_get_serialization_format(),
                ""
            }
        );

        writer_->create_topic(
            {
                "energy_image",
                "sensor_msgs/msg/Image",
                rmw_get_serialization_format(),
                ""
            }
        );

        // Create subscriptions
        energySub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/hardware/energy_image",
            qos,
            std::bind(&Logger::energyCB, this, _1)
        );

        thermalSub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/hardware/thermal_image",
            qos,
            std::bind(&Logger::thermalCB, this, _1)
        );
    }
    
    Logger::~Logger()
    {
        writer_->close();
    }

    void Logger::initializeParameters()
    {
        this->declare_parameter("log_prefix", rclcpp::ParameterValue("log"));
        this->declare_parameter("log_path", rclcpp::ParameterValue("/DroneWorkspace/data/"));
    }

    void Logger::energyCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        writer_->write(*msg, "energy_image", rclcpp::Node::now());
    }

    void Logger::thermalCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        writer_->write(*msg, "thermal_image", rclcpp::Node::now());
    }
} // namespace logger

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(logger::Logger)