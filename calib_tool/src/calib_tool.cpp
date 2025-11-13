#include <calib_tool/calib_tool.hpp>

using std::placeholders::_1;

namespace calib_tool
{

    Calib_tool::Calib_tool(const rclcpp::NodeOptions & options) : Node("calib_tool", options)
    {
        unsigned int in_width = 382;
        unsigned int in_height = 288;

        // Allocate frame buffers
        frame_in_ = cv::Mat(in_height, in_width, CV_16UC1);
        frame_out_ = cv::Mat(in_height, in_height, CV_8UC1);

        // Set QoS profile for node
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto px4_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/hardware/thermal_image",
            qos,
            std::bind(&Calib_tool::imageCB_cpu, this, _1)
        );

        rcSub_ = this->create_subscription<px4_msgs::msg::RcChannels>(
            "/fmu/out/rc_channels",
            px4_qos,
            std::bind(&Calib_tool::rcCB, this, _1)
        );

        save_frame_ = false;
        frame_count_ = 0;

        //std::string filePath_ = this->get_parameter("log_path").as_string();
        std::string filePath_ = "/home/hex/data/calib/";
        
    }

    Calib_tool::~Calib_tool() {}

    void Calib_tool::imageCB_cpu(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        double frame_in_min;
        double frame_in_max;
        double delta_in;


        if (save_frame_)
        {
            // Copy image into mapped frame
            memcpy(&frame_in_.data[0], &msg->data[0], msg->height * msg->step * sizeof(unsigned char));

            // Find min/max image values
            cv::minMaxIdx(frame_in_, &frame_in_min, &frame_in_max);
            delta_in = frame_in_max - frame_in_min;

            // Convert from 16 bit unsigned int to 8 bit unsigned int, scale to fit 8 bit range based on image min/max
            frame_in_.convertTo(frame_out_, CV_8UC1, 255. / delta_in, -frame_in_min * 255. / delta_in);

            writeString_ = filePath_ + prefix_ + std::to_string(frame_count_) + ".png";

            RCLCPP_INFO(this->get_logger(), writeString_.c_str());

            // Save the frame
            cv::imwrite(writeString_, frame_out_);

            frame_count_ ++;
            save_frame_ = false;
        }
    }

    void Calib_tool::rcCB(const px4_msgs::msg::RcChannels::UniquePtr & msg)
    {
        double saveFrame = msg->channels[8];

        if (!save_frame_ && saveFrame > 0)
        {
            save_frame_ = true;
        }
    }
    
} // namespace calib_tool

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(calib_tool::Calib_tool)