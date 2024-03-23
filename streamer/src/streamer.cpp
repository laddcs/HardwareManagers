#include <streamer/streamer.hpp>

using std::placeholders::_1;

namespace streamer
{
    Streamer::Streamer(const rclcpp::NodeOptions & options) : Node("streamer", options)
    {
        
        unsigned int in_width = 382;
        unsigned int in_height = 288;

        unsigned int out_width = in_width * 2;
        unsigned int out_height = in_height * 2;

        size_t in_pixels = in_width * in_height * sizeof(unsigned  long);
        size_t out_pixels = out_width * out_height * 3 * sizeof(unsigned  char);

        void *unified_frame_in_ptr;
        void *unified_frame_out_ptr;

        // Allocate unified buffers
        cudaMallocManaged(&unified_frame_in_ptr, in_pixels);
        cudaMallocManaged(&unified_frame_out_ptr, out_pixels);

        frame_in_ = cv::Mat(in_height, in_width, CV_16UC1, unified_frame_in_ptr);
        d_frame_in_ = cv::cuda::GpuMat(in_height, in_width, CV_16UC1, unified_frame_in_ptr);

        d_frame_convert_ = cv::cuda::GpuMat(in_height, in_width, CV_8UC1);
        d_frame_resize_ = cv::cuda::GpuMat(out_height, out_width, CV_8UC1);

        frame_out_ = cv::Mat(out_height, out_width, CV_8UC3, unified_frame_out_ptr);
        d_frame_out_ = cv::cuda::GpuMat(out_height, out_width, CV_8UC3, unified_frame_out_ptr);

        // Set QoS profile for node
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        std::string pipeline = "appsrc ! \
            video/x-raw, format=BGR, width=764, height=576, framerate=30/1 ! \
            videoconvert ! \
            video/x-raw, format=I420 ! \
            queue ! \
            x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! \
            h264parse ! \
            rtph264pay ! \
            udpsink host=0.0.0.0 port=5602";
        
        writer_ = cv::VideoWriter(pipeline, cv::CAP_GSTREAMER, 0, 30.0, cv::Size(out_width, out_height), true);
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
        // Release the gstreamer pipeline
        writer_.release();
    }

    void Streamer::imageCB(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        // Copy image into mapped frame
        memcpy(frame_in_.data, &(msg->data[0]), msg->height * msg->width * sizeof(unsigned long));

        // Convert from 16 bit unsigned int to 8 bit unsigned int
        d_frame_in_.convertTo(d_frame_convert_, CV_8UC1, 1/255);

        // Double the image size (Video writer throws a fit if the image is too small)
        cv::cuda::resize(d_frame_convert_, d_frame_resize_, cv::Size(msg->width * 2, msg->height * 2));

        // Covnert to BGR colorspace
        cv::cuda::cvtColor(d_frame_resize_, d_frame_out_, cv::COLOR_GRAY2BGR, 3);

        writer_.write(frame_out_);
    }
} // namespace streamer

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(streamer::Streamer)