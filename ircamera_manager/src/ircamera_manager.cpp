#include <ircamera_manager/ircamera_manager.hpp>

namespace ircamera_manager
{
    IRCameraManager::IRCameraManager(const rclcpp::NodeOptions & options) : Node("ircamera_manager", options)
    {
        initializeParameters();

        initializeIRDevice();

        // Initialize the device thread runner
        run_ = true;
        deviceThread_ = new std::thread(&IRCameraManager::deviceThreadRunner, this);
    }

    void IRCameraManager::deviceThreadRunner()
    {
        int serializedImages = 0;
        int chunk = 1;
        double timestamp;

        auto durInSec = std::chrono::duration<double>(1.0/imager_.getMaxFramerate());
        while(run_)
        {
            int retVal = dev_->getFrame(bufferRaw_, &timestamp);
            if(retVal == evo::IRIMAGER_SUCCESS)
            {
                imager_.process(bufferRaw_);

                if(writer_->canDoWriteOperations())
                {
                    writer_->write(timestamp, bufferRaw_, chunk, dev_->getRawBufferSize(), nmea_);
                }

                if((++serializedImages)%1000==0)
                {
                    chunk++;
                }
            }
            if(retVal == evo::IRIMAGER_DISCONNECTED)
            {
                return;
            } 

            std::this_thread::sleep_for(durInSec);
        }
    }

    void IRCameraManager::onThermalFrame(unsigned short* image, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg)
    {

    }

    void IRCameraManager::initializeIRDevice()
    {
        std::string xmlPath = this->get_parameter("device_xml_config").as_string();
        evo::IRDeviceParamsReader::readXML(xmlPath.c_str(), params_);

        RCLCPP_INFO(this->get_logger(), "Read XML Parameters!");

        dev_ = evo::IRDevice::IRCreateDevice(params_);

        RCLCPP_INFO(this->get_logger(), "IR Device Initialized!");
    }

    void IRCameraManager::initializeFileWriter()
    {
        // Initialize gps header as zeros
        memset(nmea_, 0, GPSBUFFERSIZE*sizeof(*nmea_));
    }

    void IRCameraManager::initializeParameters()
    {
        this->declare_parameter("device_xml_config", rclcpp::ParameterValue(""));
        this->declare_parameter("raw_file_path", rclcpp::ParameterValue(""));
    }
} // namespace ircamera_manager

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ircamera_manager::IRCameraManager)