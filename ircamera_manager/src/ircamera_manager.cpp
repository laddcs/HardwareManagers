#include <ircamera_manager/ircamera_manager.hpp>

using std::placeholders::_1;

namespace ircamera_manager
{
    IRCameraManager::IRCameraManager(const rclcpp::NodeOptions & options) : Node("ircamera_manager", options)
    {
        initializeParameters();

        initializeIRDevice();

        auto video_qos = rclcpp::QoS(
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
        video_qos.reliability(reliability_policy);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto px4_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Initialize thermal pub
        thermalPub_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image", video_qos);
        thermalImage_.header.frame_id = "ircamera";
        thermalImage_.height = imager_->getHeight();
        thermalImage_.width = imager_->getWidth();
        thermalImage_.encoding = "mono16";
        thermalImage_.step = thermalImage_.width*2;
        thermalImage_.data.resize(thermalImage_.height * thermalImage_.step);

        // Initialize energy pub
        energyPub_ = this->create_publisher<sensor_msgs::msg::Image>("energy_image", video_qos);
        energyImage_.header.frame_id = "ircamera";
        energyImage_.height = imager_->getHeight();
        energyImage_.width = imager_->getWidth();
        energyImage_.encoding = "mono16";
        energyImage_.step = energyImage_.width*2;
        energyImage_.data.resize(energyImage_.height * energyImage_.step);
        energyBuffer_ = new unsigned short[energyImage_.height * energyImage_.width];

        // Initialize general publishers
        flagPub_ = this->create_publisher<hardware_msgs::msg::Flag>("ir_flag", px4_qos);

        // Initialize RC Sub
        rcSub_ = this->create_subscription<px4_msgs::msg::RcChannels>(
            "/fmu/out/rc_channels",
            px4_qos,
            std::bind(&IRCameraManager::rcCB, this, _1)
        );

        dev_->startStreaming();
        RCLCPP_INFO(this->get_logger(), "Started Streaming!");

        cameraTempState_ = cameraTempRange::LOW;
        cameraResetState_ = cameraReset::ON;

        resetFlag_ = false;

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
        auto durInSec = std::chrono::duration<double>(1.0/imager_->getMaxFramerate());
        double timestamp;
        
        while(run_)
        {
            int retVal = dev_->getFrame(bufferRaw_, &timestamp);

            if(retVal == evo::IRIMAGER_SUCCESS)
            {
                imager_->process(bufferRaw_);
            }
            if(retVal == evo::IRIMAGER_DISCONNECTED)
            {
                return;
            } 
            
            std::this_thread::sleep_for(durInSec);

            //RCLCPP_INFO(this->get_logger(), "RetVal: %i", retVal);
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
        (void) arg;
    }

    void IRCameraManager::onFlagStateChange(evo::EnumFlagState flagstate, void* arg)
    {
        hardware_msgs::msg::Flag msg;
        
        msg.header.stamp = rclcpp::Node::now();
        msg.header.frame_id = "ircamera";
        msg.flag_state = flagstate;
        msg.temp_range = cameraTempState_;

        // Set the internal flagstate
        flagstate_ = flagstate;

        flagPub_->publish(msg);
    }

    void IRCameraManager::onProcessExit(void* arg)
    {
        (void) arg;
    }

    void IRCameraManager::rcCB(const px4_msgs::msg::RcChannels::UniquePtr & msg)
    {
        // If the flag is not open, do not attempt to change the temp range
        if (flagstate_ != evo::EnumFlagState::irFlagOpen)
        {
            return;
        }

        double rcCommand = msg->channels[msg->function[msg->FUNCTION_AUX_2]];

        bool validParam = false;

        // Low Range
        if ((rcCommand < 0) && (cameraTempState_ != cameraTempRange::LOW))
        {
            validParam = imager_->setTempRange(-20, 100);
            if (validParam)
            {
                imager_->forceFlagEvent(1000.f);
                cameraTempState_ = cameraTempRange::LOW;
                RCLCPP_INFO(this->get_logger(), "Temperature Range: LOW!");
            }
        }

        // Mid Range
        if ((rcCommand == 0) && (cameraTempState_ != cameraTempRange::MID))
        {
            validParam = imager_->setTempRange(0, 250);
            if (validParam)
            {
                imager_->forceFlagEvent(1000.f);
                cameraTempState_ = cameraTempRange::MID;
                RCLCPP_INFO(this->get_logger(), "Temperature Range: MID!");
            }
        }

        // High Range
        if ((rcCommand > 0) && (cameraTempState_ != cameraTempRange::HIGH))
        {
            validParam = imager_->setTempRange(150, 900);
            if (validParam)
            {
                imager_->forceFlagEvent(1000.f);
                cameraTempState_ = cameraTempRange::HIGH;
                RCLCPP_INFO(this->get_logger(), "Temperature Range: HIGH!");
            }
        }
    }

    void IRCameraManager::initializeIRDevice()
    {
        std::string logPath = this->get_parameter("logger_file_path").as_string();
        std::string logName = this->get_parameter("logger_file_name").as_string();
        std::string logfile = logPath + logName;

        evo::IRLogger::setVerbosity(evo::IRLOG_ERROR, evo::IRLOG_DEBUG, logfile.c_str());

        std::string xmlPath = this->get_parameter("device_xml_config").as_string();
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

    void IRCameraManager::initializeParameters()
    {
        this->declare_parameter("device_xml_config", rclcpp::ParameterValue("/DroneWorkspace/HardwareManagers/ircamera_manager/config/generic.xml"));
        this->declare_parameter("logger_file_name", rclcpp::ParameterValue("camera_log"));
        this->declare_parameter("logger_file_path", rclcpp::ParameterValue("/DroneWorkspace/data/"));
    }
} // namespace ircamera_manager

// Register the node as a component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ircamera_manager::IRCameraManager)