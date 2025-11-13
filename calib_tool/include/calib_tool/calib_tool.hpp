#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <px4_msgs/msg/rc_channels.hpp>


namespace calib_tool
{

class Calib_tool : public rclcpp::Node
{
    public:
        explicit Calib_tool(const rclcpp::NodeOptions & options);
        ~Calib_tool();

    private:
        cv_bridge::CvImagePtr image_ptr_;

        cv::Mat frame_in_;

        cv::Mat frame_out_;

        bool save_frame_;
        int frame_count_;

        std::string filePath_;
        std::string writeString_;
        std::string prefix_ = "image_";

        // Image Subscriber
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
        rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rcSub_;

        void imageCB_cpu(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void rcCB(const px4_msgs::msg::RcChannels::UniquePtr & msg);
};
    
}