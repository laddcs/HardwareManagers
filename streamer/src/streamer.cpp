#include <streamer/streamer.hpp>

using std::placeholders::_1;

namespace streamer
{
    Streamer::Streamer(const rclcpp::NodeOptions & options) : Node("streamer", options)
    {
        
        unsigned int width = 382;
        unsigned int height = 288;

        size_t in_pixels = width * height * sizeof(unsigned short);
        size_t out_pixels = width * height * 3 * sizeof(unsigned  char);

        // Allocate unified buffers
        cudaMallocManaged(&unified_frame_in_ptr_, in_pixels);
        cudaMallocManaged(&unified_frame_out_ptr_, out_pixels);

        frame_in_ = cv::Mat(height, width, CV_16U, unified_frame_in_ptr_);
        d_frame_in_ = cv::cuda::GpuMat(height, width, CV_16U, unified_frame_in_ptr_);

        frame_out_ = cv::Mat(height, width, CV_8UC3, unified_frame_out_ptr_);
        d_frame_out_ = cv::cuda::GpuMat(height, width, CV_8UC3, unified_frame_out_ptr_);

        // Set QoS profile for node
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        std::string pipeline = "appsrc ! \
            video/x-raw, format=RGB, width=382, height=288, framerate=30/1 ! \
            videoconvert ! \
            video/x-raw, format=I420 ! \
            queue ! \
            x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! \
            h264parse ! \
            rtph264pay ! \
            udpsink host=host.docker.internal port=5602";
        
        writer_ = cv::VideoWriter(pipeline, cv::CAP_GSTREAMER, 0, 30.0, cv::Size(382, 288), true);
        if (writer_.isOpened())
        {
            RCLCPP_INFO(this->get_logger(), "Stream Started");
        } else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed To Start Stream");
            return;
        }

        // Create subscribers
        imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/hardware/thermal_image",
            qos,
            std::bind(&Streamer::imageCB, this, _1)
        );
    }

    Streamer::~Streamer()
    {
        // Free unified buffers
        cudaFree(unified_frame_in_ptr_);
        cudaFree(unified_frame_out_ptr_);

        // Release the gstreamer pipeline
        writer_.release();
    }

    void Streamer::imageCB(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        image_ptr_ = cv_bridge::toCvCopy(msg, msg->encoding);
        image_ptr_->image.copyTo(frame_in_);

        // Perform colorspace conversion using gpu
        cv::cuda::cvtColor(d_frame_in_, d_frame_out_, cv::COLOR_GRAY2RGB, 0);

        writer_.write(frame_out_);
    }
} // namespace streamer

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(streamer::Streamer)