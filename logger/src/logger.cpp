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

        // Create Subscriptions
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

        px4VehicleOdometrySub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            qos,
            std::bind(&Logger::px4VehicleOdometryCB, this, _1)
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

    void Logger::thermalCB(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        if (!bagOpen_) {return;}
        writer_->write(msg, "/hardware/thermal_image", "sensor_msgs/msg/Image", rclcpp::Node::now());
    }

    void Logger::flagCB(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        if (!bagOpen_) {return;}
        writer_->write(msg, "/hardware/flag_state", "hardware_msgs/msg/Flag", rclcpp::Node::now());
    }

    void Logger::px4StatusCB(const px4_msgs::msg::VehicleStatus::ConstSharedPtr msg)
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
        if ((msg->arming_state != msg->ARMING_STATE_ARMED) && bagOpen_)
        {
            RCLCPP_INFO(this->get_logger(), "Closing Bag!");
            bagOpen_ = false;
            
            // This should work with fixed rosbag2 repo
            writer_->close();
            return;
        }
    }

    void Logger::px4VehicleOdometryCB(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        if (!bagOpen_) {return;}
        writer_->write(msg, "/hardware/vehicle_odometry", "px4_msgs/msg/VehicleOdometry", rclcpp::Node::now());
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
                "/hardware/thermal_image",
                "sensor_msgs/msg/Image",
                rmw_get_serialization_format(),
                ""
            }
        );

        writer_->create_topic(
            {
                "/hardware/flag_state",
                "hardware_msgs/msg/Flag",
                rmw_get_serialization_format(), 
                ""
            }
        );

        writer_->create_topic(
            {
                "/hardware/vehicle_status",
                "px4_msgs/msg/VehicleStatus",
                rmw_get_serialization_format(),
                ""
            }
        );

        writer_->create_topic(
            {
                "/hardware/vehicle_odometry",
                "px4_msgs/msg/VehicleOdometry",
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