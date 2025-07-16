#include <streamer/streamer.hpp>

using std::placeholders::_1;

namespace streamer
{
    Streamer::Streamer(const rclcpp::NodeOptions & options) : Node("streamer", options)
    {
        
        unsigned int in_width = 382;
        unsigned int in_height = 288;

        // Double size to fix videoWriter issue
        unsigned int out_width = in_width * 2;
        unsigned int out_height = in_height * 2;

        size_t in_pixels = in_width * in_height * sizeof(unsigned  short); // 16 bit step
        size_t out_pixels = out_width * out_height * sizeof(unsigned  char); // 8 bit step

        // Get frame processing strategy from launch parameters
        this->declare_parameter("imgproc", rclcpp::ParameterValue("0"));
        int imgproc = this->get_parameter("imgproc").as_int();

        // Allocate frame buffers
        if (imgproc == Imgproc::CPU)
        {
            frame_in_ = cv::Mat(in_height, in_width, CV_16UC1);
            frame_convert_ = cv::Mat(in_height, in_width, CV_8UC1);
            frame_out_ = cv::Mat(out_height, out_width, CV_8UC1);
        } 
        else if (imgproc == Imgproc::GPU)
        {
            void *unified_frame_in_ptr;
            void *unified_frame_out_ptr;

            // Allocate unified buffers
            cudaMallocManaged(&unified_frame_in_ptr, in_pixels);
            cudaMallocManaged(&unified_frame_out_ptr, out_pixels);

            // Create image processing containers using unified buffers
            frame_in_ = cv::Mat(in_height, in_width, CV_16UC1, unified_frame_in_ptr);
            d_frame_in_ = cv::cuda::GpuMat(in_height, in_width, CV_16UC1, unified_frame_in_ptr);

            d_frame_convert_ = cv::cuda::GpuMat(in_height, in_width, CV_8UC1);

            frame_out_ = cv::Mat(out_height, out_width, CV_8UC1, unified_frame_out_ptr);
            d_frame_out_ = cv::cuda::GpuMat(out_height, out_width, CV_8UC1, unified_frame_out_ptr);
        }

        // Set QoS profile for node
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Need to make this more modular, udp packing / encoding params as ros params?
        std::string pipeline = "appsrc ! \
            video/x-raw, format=GRAY8, width=764, height=576, framerate=30/1 ! \
            videoconvert ! \
            video/x-raw, format=I420 ! \
            queue ! \
            x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! \
            h264parse ! \
            rtph264pay ! \
            udpsink host=0.0.0.0 port=5602";
        
        writer_ = cv::VideoWriter(pipeline, cv::CAP_GSTREAMER, 0, 30.0, cv::Size(out_width, out_height), false);
        if (writer_.isOpened())
        {
            RCLCPP_INFO(this->get_logger(), "Stream Started");
        } else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed To Start Stream");
            return;
        }

        // Create subscribers
        if (imgproc == Imgproc::CPU)
        {
            imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/hardware/thermal_image",
                qos,
                std::bind(&Streamer::imageCB_cpu, this, _1)
            );
        }
        else if (imgproc == Imgproc::GPU)
        {
            imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/hardware/thermal_image",
                qos,
                std::bind(&Streamer::imageCB_gpu, this, _1)
            );
        }
    }

    Streamer::~Streamer()
    {
        // Release the gstreamer pipeline
        writer_.release();
    }

    void Streamer::imageCB_cpu(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        double frame_in_min;
        double frame_in_max;
        double delta_in;

        // Copy image into mapped frame
        memcpy(&frame_in_.data[0], &msg->data[0], msg->height * msg->step * sizeof(unsigned char));

        // Find min/max image values
        cv::minMaxIdx(frame_in_, &frame_in_min, &frame_in_max);
        delta_in = frame_in_max - frame_in_min;

        // Convert from 16 bit unsigned int to 8 bit unsigned int, scale to fit 8 bit range based on image min/max
        frame_in_.convertTo(frame_convert_, CV_8UC1, 255. / delta_in, -frame_in_min * 255. / delta_in);

        // Double the image size (Video writer throws a fit if the image is too small)
        cv::resize(frame_convert_, frame_out_, cv::Size(msg->width * 2, msg->height * 2));

        writer_->write(frame_out_);
    }

    void Streamer::imageCB_gpu(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        double frame_in_min;
        double frame_in_max;
        double delta_in;

        // Copy image into mapped frame
        memcpy(&frame_in_.data[0], &msg->data[0], msg->height * msg->step * sizeof(unsigned char));

        // Find min/max image values
        cv::cuda::minMax(d_frame_in_, &frame_in_min, &frame_in_max);
        delta_in = frame_in_max - frame_in_min;

        // Convert from 16 bit unsigned int to 8 bit unsigned int, scale to fit 8 bit range based on image min/max
        d_frame_in_.convertTo(d_frame_convert_, CV_8UC1, 255. / delta_in, -frame_in_min * 255. / delta_in);

        // Double the image size (Video writer throws a fit if the image is too small)
        cv::cuda::resize(d_frame_convert_, d_frame_out_, cv::Size(msg->width * 2, msg->height * 2));

        writer_.write(frame_out_);
    }
} // namespace streamer

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(streamer::Streamer)