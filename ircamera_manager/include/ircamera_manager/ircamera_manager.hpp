#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <libirimager/IRDevice.h>
#include <libirimager/IRImager.h>
#include <libirimager/IRImagerClient.h>
#include <libirimager/IRLogger.h>
#include <libirimager/IRFileWriter.h>

namespace ircamera_manager
{

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
        // IR device parameters object, used to initialize device and imager
        evo::IRDeviceParams params_;

        // Header for raw data stream
        evo::RawdataHeader header_;

        // Pointer to IR device manager
        evo::IRDevice* dev_;

        // Optris image processing pipeline
        evo::IRImager imager_;

        // Writes raw serialized data stream from IRDevice to disk, this is the most efficient way to store the data (without losing precision)
        std::shared_ptr<evo::IRFileWriter> writer_;

        unsigned char* bufferRaw_;
        char nmea_[GPSBUFFERSIZE];
        bool run_;
        std::thread* deviceThread_;

        // Thermal image publishing
        sensor_msgs::msg::Image thermalImage_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermalPub_;

        // Loop that checks for a new IR image and passes it to the image processing pipeline
        // Executed in the deviceThread_
        void deviceThreadRunner();
        
        // Checks xml parameters, initializes the IR device with given paramters
        void initializeIRDevice();

        // Initializes the IR file writer
        void initializeFileWriter();

        // Initialize node parameters
        void initializeParameters();
};

} // namespace ircamera_manager