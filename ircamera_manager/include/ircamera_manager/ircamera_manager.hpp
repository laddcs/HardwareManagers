#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <hardware_msgs/msg/flag.hpp>

#include <libirimager/IRDevice.h>
#include <libirimager/IRImager.h>
#include <libirimager/IRImagerClient.h>
#include <libirimager/IRLogger.h>

namespace ircamera_manager
{

enum cameraTempRange
{
    LOW,
    MID,
    HIGH
};

class IRCameraManager : public rclcpp::Node, public evo::IRImagerClient
{
    public:
        explicit IRCameraManager(const rclcpp::NodeOptions & options);
        virtual ~IRCameraManager();

        virtual void onRawFrame(unsigned char* data, int size) {(void)data; (void)size;};

        // Called on thermal frame recieved
        virtual void onThermalFrame(unsigned short* image, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg);

        virtual void onVisibleFrame(unsigned char* image, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg);

        virtual void onFlagStateChange(evo::EnumFlagState flagstate, void* arg);

        virtual void onProcessExit(void* arg);


    private:

        // Current Flag State
        evo::EnumFlagState flagstate_;
        cameraTempRange cameraTempState_;
        bool resetFlag_;

        // IR device parameters object, used to initialize device and imager
        evo::IRDeviceParams params_;

        // Pointer to IR device manager
        evo::IRDevice* dev_;

        // Optris image processing pipeline
        evo::IRImager* imager_;

        unsigned char* bufferRaw_;
        bool run_;
        std::thread* deviceThread_;

        // Thermal image publishing
        sensor_msgs::msg::Image thermalImage_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermalPub_;

        // Energy image publishing
        unsigned short * energyBuffer_;
        sensor_msgs::msg::Image energyImage_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr energyPub_;

        // General Publishers
        rclcpp::Publisher<hardware_msgs::msg::Flag>::SharedPtr flagPub_;

        //Subscribers
        rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rcSub_;

        // Loop that checks for a new IR image and passes it to the image processing pipeline
        // Executed in the deviceThread_
        void deviceThreadRunner();
        
        // Checks xml parameters, initializes the IR device with given paramters
        void initializeIRDevice();

        // Initializes the IR file writer
        void initializeFileWriter();

        // Initialize node parameters
        void initializeParameters();

        //Subscriber Callbacks
        void rcCB(const px4_msgs::msg::RcChannels::UniquePtr & msg);
};

} // namespace ircamera_manager