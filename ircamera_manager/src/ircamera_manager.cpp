#include <ircamera_manager/ircamera_manager.hpp>

namespace ircamera_manager
{
    IRCameraManager::IRCameraManager(const rclcpp::NodeOptions & options) : Node("ircamera_manager", options)
    {
        initializeParameters();

        initializeIRDevice();

        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
                // The history policy determines how messages are saved until taken by
                // the reader.
                // KEEP_ALL saves all messages until they are taken.
                // KEEP_LAST enforces a limit on the number of messages that are saved,
                // specified by the "depth" parameter.
                RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                // The next parameter represents how many messages to store in history when the
                // history policy is KEEP_LAST.
                1
            ));

        // The reliability policy can be reliable, meaning that the underlying transport layer will try
        // ensure that every message gets received in order, or best effort, meaning that the transport
        // makes no guarantees about the order or reliability of delivery.
        // Options are: SYSTEM_DEFAULT, RELIABLE, BEST_EFFORT and UNKNOWN
        rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
        qos.reliability(reliability_policy);

        // Initialize thermal pub
        thermalPub_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image", qos);
        thermalImage_.header.frame_id = "ircamera";
        thermalImage_.height = imager_->getHeight();
        thermalImage_.width = imager_->getWidth();
        thermalImage_.encoding = "mono16";
        thermalImage_.step = thermalImage_.width*2;
        thermalImage_.data.resize(thermalImage_.height * thermalImage_.step);

        // Initialize energy pub
        energyPub_ = this->create_publisher<sensor_msgs::msg::Image>("energy_image", qos);
        energyImage_.header.frame_id = "ircamera";
        energyImage_.height = imager_->getHeight();
        energyImage_.width = imager_->getWidth();
        energyImage_.encoding = "mono16";
        energyImage_.step = energyImage_.width*2;
        energyImage_.data.resize(energyImage_.height * energyImage_.step);
        energyBuffer_ = new unsigned short[energyImage_.height * energyImage_.width];

        dev_->startStreaming();
        RCLCPP_INFO(this->get_logger(), "Started Streaming!");

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
        delete [] energyBuffer_;
    }

    void IRCameraManager::deviceThreadRunner()
    {
        int serializedImages = 0;
        int chunk = 1;
        double timestamp;

        auto durInSec = std::chrono::duration<double>(1.0/imager_->getMaxFramerate());
        while(run_)
        {
            int retVal = dev_->getFrame(bufferRaw_, &timestamp);

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
                
        // Publish thermal image
        memcpy(&thermalImage_.data[0], image, w*h*sizeof(*image));
        thermalImage_.header.stamp = rclcpp::Node::now();
        thermalPub_->publish(thermalImage_);

        // Publish energy image
        imager_->getEnergyBuffer(energyBuffer_);
        memcpy(&energyImage_.data[0], energyBuffer_, w*h*sizeof(*energyBuffer_));
        energyImage_.header.stamp = rclcpp::Node::now();
        energyPub_->publish(energyImage_);
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
        std::string xmlPath = "/DroneWorkspace/HardwareManagers/ircamera_manager/config/generic.xml";
        evo::IRDeviceParamsReader::readXML(xmlPath.c_str(), params_);

        RCLCPP_INFO(this->get_logger(), "Read XML Parameters!");

        dev_ = evo::IRDevice::IRCreateDevice(params_);

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