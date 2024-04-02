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

        bagOpen_ = false;
        bagNum_ = 0;

        // Set QoS profile for node
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        writer_ = std::make_unique<rosbag2_cpp::Writer>();

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

        flagSub_ = this->create_subscription<hardware_msgs::msg::Flag>(
            "/hardware/ir_flag",
            qos,
            std::bind(&Logger::flagCB, this, _1)
        );

        px4StatusSub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status",
            qos,
            std::bind(&Logger::px4StatusCB, this, _1)
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

    void Logger::energyCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
    {
        if (!bagOpen_) {return;}
        writer_->write(*msg, "energy_image", rclcpp::Node::now());
    }

    void Logger::thermalCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
    {
        if (!bagOpen_) {return;}
        writer_->write(*msg, "thermal_image", rclcpp::Node::now());
    }

    void Logger::flagCB(const hardware_msgs::msg::Flag::ConstSharedPtr & msg) const
    {
        if (!bagOpen_) {return;}
        writer_->write(*msg, "flag_state", rclcpp::Node::now());
    }

    void Logger::px4StatusCB(const px4_msgs::msg::VehicleStatus::ConstPtr msg)
    {
        // Open the bag on arming
        if ((msg->arming_state == msg->ARMING_STATE_ARMED) && !bagOpen_)
        {
            RCLCPP_INFO(this->get_logger(), "Opening Bag!");
            openBag();
            bagOpen_ = true;
            return;
        }

        // Close the bag on disarm
        if ((msg->arming_state == msg->ARMING_STATE_SHUTDOWN) && bagOpen_)
        {
            RCLCPP_INFO(this->get_logger(), "Closing Bag!");
            bagOpen_ = false;
            writer_->close();
            auto writer = std::make_unique<rosbag2_cpp::Writer>();
            writer_.reset(writer.get());
            return;
        }
    }

    void Logger::openBag()
    {
        std::string filePath = this->get_parameter("log_path").as_string();
        std::string filePrefix = this->get_parameter("log_prefix").as_string();
        std::string fileNum = "_" + std::to_string(bagNum_);
        std::string logFile = filePath + filePrefix + fileNum;
        
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

        writer_->create_topic(
            {
                "flag_state",
                "hardware_msgs/msg/Flag",
                rmw_get_serialization_format(), 
                ""
            }
        );

        bagNum_++;
    }

} // namespace logger

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(logger::Logger)