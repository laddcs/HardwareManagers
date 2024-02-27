#include <ircamera_manager/ircamera_manager.hpp>

namespace ircamera_manager
{
    IRCameraManager::IRCameraManager(const rclcpp::NodeOptions & options) : Node("ircamera_manager", options)
    {
        initializeParameters();

        initializeIRDevice();

        if(dev_->startStreaming()!=0)
        {
            RCLCPP_INFO(this->get_logger(), "Started Streaming!");
        } else
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to start streaming");
            return;
        }

        // Initialize the device thread runner
        run_ = true;
        deviceThread_ = new std::thread(&IRCameraManager::deviceThreadRunner, this);
    }

    IRCameraManager::~IRCameraManager()
    {
        run_ = false;
        deviceThread_->join();
        dev_->stopStreaming();

        delete [] bufferRaw_;
    }

    void IRCameraManager::deviceThreadRunner()
    {
        int serializedImages = 0;
        int chunk = 1;
        double timestamp;

        auto durInSec = std::chrono::duration<double>(1.0/imager_->getMaxFramerate());
        while(run_)
        {
            RCLCPP_INFO(this->get_logger(), "Reading Device!");

            int retVal = dev_->getFrame(bufferRaw_, &timestamp);

            RCLCPP_INFO(this->get_logger(), "Retval: %i", retVal);

            if(retVal == evo::IRIMAGER_SUCCESS)
            {
                imager_->process(bufferRaw_);
/*
                if(writer_->canDoWriteOperations())
                {
                    writer_->write(timestamp, bufferRaw_, chunk, dev_->getRawBufferSize(), nmea_);
                }

                if((++serializedImages)%1000==0)
                {
                    chunk++;
                }
*/
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
        (void) arg;
    }

    void IRCameraManager::onVisibleFrame(unsigned char* image, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg)
    {

    }

    void IRCameraManager::onFlagStateChange(evo::EnumFlagState flagstate, void* arg)
    {

    }

    void IRCameraManager::onProcessExit(void* arg)
    {

    }

    void IRCameraManager::initializeIRDevice()
    {
        std::string xmlPath = this->get_parameter("device_xml_config").as_string();
        evo::IRDeviceParamsReader::readXML(xmlPath.c_str(), params_);

        RCLCPP_INFO(this->get_logger(), "Read XML Parameters!");

        dev_ = evo::IRDevice::IRCreateDevice(params_);
        dev_->setClient(this);

        bufferRaw_ = new unsigned char[dev_->getRawBufferSize()];

        RCLCPP_INFO(this->get_logger(), "IR Device Initialized!");

        imager_ = new evo::IRImager();

        if(imager_->init(&params_, dev_->getFrequency(), dev_->getWidth(), dev_->getHeight(), dev_->controlledViaHID()))
        {
            imager_->setClient(this);
        }else
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to initialize imager");
        }
        
    }

    void IRCameraManager::initializeFileWriter()
    {
        // Initialize gps header as zeros
        memset(nmea_, 0, GPSBUFFERSIZE*sizeof(*nmea_));
    }

    void IRCameraManager::initializeParameters()
    {
        this->declare_parameter("device_xml_config", rclcpp::ParameterValue("/DroneWorkspace/HardwareManagers/ircamera_manager/config/generic.xml"));
        this->declare_parameter("raw_file_path", rclcpp::ParameterValue(""));
    }
} // namespace ircamera_manager

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ircamera_manager::IRCameraManager)