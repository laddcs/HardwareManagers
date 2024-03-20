#include <streamer/streamer.hpp>

using std::placeholders::_1;

namespace streamer
{
    Streamer::Streamer(const rclcpp::NodeOptions & options) : Node("streamer", options)
    {
        // Set QoS profile for node
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        std::string pipeline = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=127.0.0.1 port=5000";
        writer_ = cv::VideoWriter(pipeline, cv::CAP_GSTREAMER, 0, 30.0, cv::Size(382, 288), true);
        if (writer_.isOpened())
        {
            RCLCPP_INFO(this->get_logger(), "Stream Started");
        } else
        {
            RCLCPP_INFO(this->get_logger(), "Failed To Start Stream");
        }

        imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/hardware/thermal_image",
            qos,
            std::bind(&Streamer::imageCB, this, _1)
        );
    }

    void Streamer::imageCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        image_ptr_ = cv_bridge::toCvCopy(*msg, msg->encoding);

        //src_.upload(image_ptr_->image);

        //cv::gpu::cvtColor(src_, dst_, cv::COLOR_GRAY2BGR, 0);

        writer_.write(image_ptr_->image);
    }
} // namespace streamer'

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(streamer::Streamer)