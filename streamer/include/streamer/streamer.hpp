#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include <sensor_msgs/msg/image.hpp>


namespace streamer
{

class Streamer : public rclcpp::Node
{
    public:
        explicit Streamer(const rclcpp::NodeOptions & options);

    private:

        cv_bridge::CvImagePtr image_ptr_;
        cv::cuda::GpuMat src_, dst_;

        // Image Subscriber
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;

        // Video Streamer
        cv::VideoWriter writer_;

        void imageCB(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
};

} // namespace streamer