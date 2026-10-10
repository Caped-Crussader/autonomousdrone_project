#include "falcon_manager_node.hpp"


FalconManagerNode::FalconManagerNode() : Node("falcon_manager_node") {
    
    falconManager_ = Sds::Falcon::createFalconManager();

    RCLCPP_INFO(this->get_logger(), "Falcon Manager Created");
    
    /* ------------------------- Create Services ------------------------- */
    
    get_version_service_ = this->create_service<falcon_pkg::srv::GetVersion>(
        "get_version",
        std::bind(&FalconManagerNode::handleGetVersion, this, std::placeholders::_1, std::placeholders::_2)
    );

    get_firmware_version_service_ = this->create_service<falcon_pkg::srv::GetFirmwareVersion>(
        "get_firmware_version",
        std::bind(&FalconManagerNode::handleGetFirmwareVersion, this, std::placeholders::_1, std::placeholders::_2)
    );

    initialize_network_service_ = this->create_service<falcon_pkg::srv::InitializeNetwork>(
        "initialize_network",
        std::bind(&FalconManagerNode::handleInitializeNetwork, this, std::placeholders::_1, std::placeholders::_2)
    );

    initialize_network_advanced_service_ = this->create_service<falcon_pkg::srv::InitializeNetworkAdvanced>(
        "initialize_network_advanced",
        std::bind(&FalconManagerNode::handleInitializeNetworkAdvanced, this, std::placeholders::_1, std::placeholders::_2)
    );

    set_tracking_mode_service_ = this->create_service<falcon_pkg::srv::SetTrackingMode>(
        "set_tracking_mode",
        std::bind(&FalconManagerNode::handleSetTrackingMode, this, std::placeholders::_1, std::placeholders::_2)
    );

    initialize_relative_beacon_service_ = this->create_service<falcon_pkg::srv::InitializeRelativeBeacon>(
        "initialize_relative_beacon",
        std::bind(&FalconManagerNode::handleInitializeRelativeBeacon, this, std::placeholders::_1, std::placeholders::_2)
    );

    initialize_relative_angle_service_ = this->create_service<falcon_pkg::srv::InitializeRelativeAngle>(
        "initialize_relative_angle",
        std::bind(&FalconManagerNode::handleInitializeRelativeAngle, this, std::placeholders::_1, std::placeholders::_2)
    );

    initialize_6dof_service_ = this->create_service<falcon_pkg::srv::Initialize6Dof>(
        "initialize_6dof",
        std::bind(&FalconManagerNode::handleInitialize6Dof, this, std::placeholders::_1, std::placeholders::_2)
    );

    swap_map_service_ = this->create_service<falcon_pkg::srv::SwapMap>(
        "swap_map",
        std::bind(&FalconManagerNode::handleSwapMap, this, std::placeholders::_1, std::placeholders::_2)
    );

    get_gyro_calibration_service_ = this->create_service<falcon_pkg::srv::GetGyroCalibration>(
        "get_gyro_calibration",
        std::bind(&FalconManagerNode::handleGetGyroCalibration, this, std::placeholders::_1, std::placeholders::_2)
    );

    load_shutter_plugin_service_ = this->create_service<falcon_pkg::srv::LoadShutterPlugin>(
        "load_shutter_plugin",
        std::bind(&FalconManagerNode::handleLoadShutterPlugin, this, std::placeholders::_1, std::placeholders::_2)
    );

    load_default_shutter_control_service_ = this->create_service<falcon_pkg::srv::LoadDefaultShutterControl>(
        "load_default_shutter_control",
        std::bind(&FalconManagerNode::handleLoadDefaultShutterControl, this, std::placeholders::_1, std::placeholders::_2)
    );


    /* ------------------------- Register Callbacks ------------------------- */

    heartbeat_publisher_ = this->create_publisher<falcon_pkg::msg::Heartbeat>("heartbeat", 10);
    falconManager_->registerHeartbeatCallback([this](const Sds::Falcon::Heartbeat& heartbeat){
        this->publishHeartbeat(heartbeat);
    });

    status_message_publisher_ = this->create_publisher<falcon_pkg::msg::StatusMessage>("status_message", 10);
    falconManager_->registerMessageCallback([this](const Sds::Falcon::StatusMessage& statusMessage){
        this->publishStatusMessage(statusMessage);
    });

    relative_beacon_publisher_ = this->create_publisher<falcon_pkg::msg::PoseRelativeBeaconCollection>("relative_beacon", 10);
    falconManager_->registerPoseRelativeBeaconCallback([this](const Sds::Falcon::PoseRelativeBeaconCollection& col){
        this->publishRelativeBeacon(col);
    });

    relative_angle_publisher_ = this->create_publisher<falcon_pkg::msg::RelativeAngleCollection>("relative_angle", 10);
    falconManager_->registerRelativeAngleCallback([this](const Sds::Falcon::RelativeAngleCollection& col){
        this->publishRelativeAngle(col);
    });

    sixdof_publisher_ = this->create_publisher<falcon_pkg::msg::Pose6Dof>("sixdof", 10);
    falconManager_->registerPose6DofCallback([this](const Sds::Falcon::Pose6Dof& sd){
        this->publish6Dof(sd);
    });

    current_shutter_publisher_ = this->create_publisher<falcon_pkg::msg::CurrentShutter>("current_shutter", 10);
    falconManager_->registerCurrentShutterCallback([this](const Sds::Falcon::CurrentShutter& cs){
        this->publishCurrentShutter(cs);
    });

    fov_report_publisher_ = this->create_publisher<falcon_pkg::msg::FieldOfViewReport>("field_of_view_report", 10);
    falconManager_->registerFieldOfViewReportCallback([this](const Sds::Falcon::FieldOfViewReport& fov){
        this->publishFieldOfViewReport(fov);
    });

    dynamic_message_publisher_ = this->create_publisher<falcon_pkg::msg::DynamicMessage>("dynamic_message", 10);
    falconManager_->registerDynamicMessageCallback([this](const Sds::Falcon::DynamicMessage& dm){
        this->publishDynamicMessage(dm);
    });
}

void FalconManagerNode::handleGetVersion(const std::shared_ptr<falcon_pkg::srv::GetVersion::Request> request,
                        std::shared_ptr<falcon_pkg::srv::GetVersion::Response> response) {
    Sds::Falcon::Version version = Sds::Falcon::getVersion();
    response->version.major = version.getMajor();
    response->version.minor = version.getMinor();
    response->version.patch = version.getPatch();
}

void FalconManagerNode::handleGetFirmwareVersion(const std::shared_ptr<falcon_pkg::srv::GetFirmwareVersion::Request> request,
                        std::shared_ptr<falcon_pkg::srv::GetFirmwareVersion::Response> response) {
    Sds::Falcon::Version version = falconManager_->getFirmwareVersion();
    response->version.major = version.getMajor();
    response->version.minor = version.getMinor();
    response->version.patch = version.getPatch();
}

void FalconManagerNode::handleInitializeNetwork(const std::shared_ptr<falcon_pkg::srv::InitializeNetwork::Request> request,
                        std::shared_ptr<falcon_pkg::srv::InitializeNetwork::Response> response) {
    networkConfig.sensor_id = request->network_config.sensor_id;
    networkConfig.sensor_network_name = request->network_config.sensor_network_name;
    networkConfig.udp_read_port = request->network_config.udp_read_port;

    response->success = falconManager_->initializeNetwork(networkConfig);
    RCLCPP_INFO(this->get_logger(), "Initialize Network - %s", (response->success ? "SUCCESS" : "FAILED"));
}

void FalconManagerNode::handleInitializeNetworkAdvanced(const std::shared_ptr<falcon_pkg::srv::InitializeNetworkAdvanced::Request> request,
                        std::shared_ptr<falcon_pkg::srv::InitializeNetworkAdvanced::Response> response) {
    networkAdvancedConfig.udp_read_port = request->network_advanced_config.udp_read_port;
    networkAdvancedConfig.sensor_id = request->network_advanced_config.sensor_id;
    networkAdvancedConfig.sensor_ip = request->network_advanced_config.sensor_ip;

    response->success = falconManager_->initializeNetwork(networkAdvancedConfig);
    RCLCPP_INFO(this->get_logger(), "Initialize Network Advanced - %s", (response->success ? "SUCCESS" : "FAILED"));
}

Sds::Falcon::TrackingMode FalconManagerNode::getTrackingMode(const std::string& modeStr) {
    if (modeStr == "Sixdof"){ 
        return Sds::Falcon::TrackingMode::Sixdof;
    } else if (modeStr == "RelativeBeacon"){ 
        return Sds::Falcon::TrackingMode::RelativeBeacon;
    } else if (modeStr == "RelativeAngle"){ 
        return Sds::Falcon::TrackingMode::RelativeAngle;
    } else if (modeStr == "TrackingOFF"){ 
        return Sds::Falcon::TrackingMode::TrackingOFF;
    } else {
        RCLCPP_INFO(this->get_logger(), "Invalid Tracking Mode string");
        return Sds::Falcon::TrackingMode::TrackingOFF;
    }
}

void FalconManagerNode::handleSetTrackingMode(const std::shared_ptr<falcon_pkg::srv::SetTrackingMode::Request> request,
                        std::shared_ptr<falcon_pkg::srv::SetTrackingMode::Response> response) {
    Sds::Falcon::TrackingMode mode = getTrackingMode(request->tracking_mode);
    falconManager_->setTrackingMode(mode);
    RCLCPP_INFO(this->get_logger(), "Tracking Mode Set - %s", Sds::Falcon::to_string(mode).c_str());
}

Sds::Falcon::ShutterSettings FalconManagerNode::getShutterSettings(uint8_t minVal, uint8_t maxVal) {
    if (minVal == 63 || maxVal == 63) {
        return Sds::Falcon::getShutterSettingsAuto();
    } else if (minVal == maxVal) {
        return Sds::Falcon::getShutterSettingsFixed(minVal);
    } else if (minVal >=0 && maxVal <=16 && minVal < maxVal) {
        return Sds::Falcon::getShutterSettingsRange(minVal, maxVal);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid shutter setting values.");
        return Sds::Falcon::getShutterSettingsAuto();
    }
}

void FalconManagerNode::handleInitializeRelativeBeacon(const std::shared_ptr<falcon_pkg::srv::InitializeRelativeBeacon::Request> request,
                        std::shared_ptr<falcon_pkg::srv::InitializeRelativeBeacon::Response> response) {
    
    uint8_t minVal = request->config.shutter_settings.min_shutter;
    uint8_t maxVal = request->config.shutter_settings.max_shutter;

    configRelB.shutter_settings = getShutterSettings(minVal, maxVal);

    configRelB.expected_beacon_ids = request->config.expected_beacon_ids; //TODO - check that this works
    configRelB.field_of_view_cutoff_deg = request->config.field_of_view_cutoff_deg;
    configRelB.rel_lights_dump_path = request->config.rel_lights_dump_path;

    response->success = falconManager_->initializeRelativeBeacon(configRelB);
    RCLCPP_INFO(this->get_logger(), "Initialize Relative Beacon Tracking Mode - %s", (response->success ? "SUCCESS" : "FAILED"));
}

void FalconManagerNode::handleInitializeRelativeAngle(const std::shared_ptr<falcon_pkg::srv::InitializeRelativeAngle::Request> request,
                        std::shared_ptr<falcon_pkg::srv::InitializeRelativeAngle::Response> response) {
    
    uint8_t minVal = request->config.shutter_settings.min_shutter;
    uint8_t maxVal = request->config.shutter_settings.max_shutter;
    
    configRelA.shutter_settings = getShutterSettings(minVal, maxVal);

    configRelA.expected_beacon_ids = request->config.expected_beacon_ids;
    configRelA.matching_mode_single_beacon = request->config.matching_mode_single_beacon; 
    configRelA.field_of_view_cutoff_deg = request->config.field_of_view_cutoff_deg;
    configRelA.rel_angles_dump_path = request->config.rel_angles_dump_path;

    response->success = falconManager_->initializeRelativeAngle(configRelA);
    RCLCPP_INFO(this->get_logger(), "Initialize Relative Angle Tracking Mode - %s", (response->success ? "SUCCESS" : "FAILED"));
}

void FalconManagerNode::handleInitialize6Dof(const std::shared_ptr<falcon_pkg::srv::Initialize6Dof::Request> request,
                        std::shared_ptr<falcon_pkg::srv::Initialize6Dof::Response> response) {
    for (const falcon_pkg::msg::Beacon& beacon_msg : request->config.map) {
        Sds::Falcon::Beacon beacon;
        beacon.x = beacon_msg.x;
        beacon.y = beacon_msg.y;
        beacon.z = beacon_msg.z;
        beacon.id = beacon_msg.id;
        config6Dof.map.push_back(beacon);
    }

    for (const falcon_pkg::msg::GyroOffset& gyro_msg : request->config.gyro_calibration) {
        Sds::Falcon::GyroOffset gyro;
        gyro.sensorId = gyro_msg.sensor_id;
        gyro.datetime = gyro_msg.datetime;
        gyro.temperature = gyro_msg.temperature;
        gyro.gyro_x = gyro_msg.gyro_x;
        gyro.gyro_y = gyro_msg.gyro_y;
        gyro.gyro_z = gyro_msg.gyro_z;
        gyro.standard_dev = gyro_msg.standard_dev;
        config6Dof.gyroCalibration.push_back(gyro);
    }
    
    config6Dof.skip_gyro_calibration = request->config.skip_gyro_calibration; 

    uint8_t minVal = request->config.shutter_settings.min_shutter;
    uint8_t maxVal = request->config.shutter_settings.max_shutter;
    
    config6Dof.shutter_settings = getShutterSettings(minVal, maxVal);

    config6Dof.no_gyro_mode = request->config.no_gyro_mode;
    config6Dof.sixdof_dump_path = request->config.sixdof_dump_path;

    response->success = falconManager_->initialize6Dof(config6Dof);
    RCLCPP_INFO(this->get_logger(), "Initialize 6DOF Tracking Mode - %s", (response->success ? "SUCCESS" : "FAILED"));
}

void FalconManagerNode::handleSwapMap(const std::shared_ptr<falcon_pkg::srv::SwapMap::Request> request,
                        std::shared_ptr<falcon_pkg::srv::SwapMap::Response> response) {
    std::vector<Sds::Falcon::Beacon> map;
    
    for (const falcon_pkg::msg::Beacon& beacon_msg : request->map) {
        Sds::Falcon::Beacon beacon;
        beacon.x = beacon_msg.x;
        beacon.y = beacon_msg.y;
        beacon.z = beacon_msg.z;
        beacon.id = beacon_msg.id;
        map.push_back(beacon);
    }

    falconManager_->swapMap(map);
    RCLCPP_INFO(this->get_logger(), "Map Swapped");
}

void FalconManagerNode::handleGetGyroCalibration(const std::shared_ptr<falcon_pkg::srv::GetGyroCalibration::Request> request,
                        std::shared_ptr<falcon_pkg::srv::GetGyroCalibration::Response> response) {
    Sds::Falcon::ConfigGyroCalibration configGyro;
    configGyro.duration_seconds = request->config_gyro.duration_seconds;

    std::vector<Sds::Falcon::GyroOffset> calibration = falconManager_->getGyroCalibration(configGyro);
    for (const auto& g_offset : calibration) {
        falcon_pkg::msg::GyroOffset g_offset_msg;
        g_offset_msg.sensor_id = g_offset.sensorId;
        g_offset_msg.datetime = g_offset.datetime;
        g_offset_msg.temperature = g_offset.temperature; 
        g_offset_msg.gyro_x = g_offset.gyro_x;
        g_offset_msg.gyro_y = g_offset.gyro_y;
        g_offset_msg.gyro_z = g_offset.gyro_z;
        g_offset_msg.standard_dev = g_offset.standard_dev;
        
        response->calibration.push_back(g_offset_msg);
    }
}

void FalconManagerNode::handleLoadShutterPlugin(const std::shared_ptr<falcon_pkg::srv::LoadShutterPlugin::Request> request,
                        std::shared_ptr<falcon_pkg::srv::LoadShutterPlugin::Response> response) {
    falconManager_->loadShutterPlugin(request->path.c_str());
    RCLCPP_INFO(this->get_logger(), "Shutter Plugin Loaded");
}

void FalconManagerNode::handleLoadDefaultShutterControl(const std::shared_ptr<falcon_pkg::srv::LoadDefaultShutterControl::Request> request,
                        std::shared_ptr<falcon_pkg::srv::LoadDefaultShutterControl::Response> response) {
    falconManager_->loadDefaultShutterControl();
    RCLCPP_INFO(this->get_logger(), "Default Shutter Control Loaded");
}

void FalconManagerNode::publishHeartbeat(const Sds::Falcon::Heartbeat& heartbeat) {
    falcon_pkg::msg::Heartbeat msg;
    msg.sensor_communication_ok = heartbeat.sensor_communication_ok;
    msg.current_tracking_mode = Sds::Falcon::to_string(heartbeat.current_tracking_mode);
    msg.current_tracking_mode_state = Sds::Falcon::to_string(heartbeat.current_tracking_mode_state);

    heartbeat_publisher_->publish(msg);
}

void FalconManagerNode::publishStatusMessage(const Sds::Falcon::StatusMessage& statusMessage) {
    falcon_pkg::msg::StatusMessage msg;
    msg.severity = (int)statusMessage.severity;
    msg.layer = (int)statusMessage.layer;
    msg.event_code = (int)statusMessage.event_code;
    msg.message = statusMessage.message.c_str();

    status_message_publisher_->publish(msg);
}

void FalconManagerNode::publishRelativeBeacon(const Sds::Falcon::PoseRelativeBeaconCollection& col) {
    falcon_pkg::msg::PoseRelativeBeaconCollection msg;

    for (const auto& beacon_pose : col) {
        falcon_pkg::msg::PoseRelativeBeacon beacon_pose_msg;
        beacon_pose_msg.x = beacon_pose.x;
        beacon_pose_msg.y = beacon_pose.y;
        beacon_pose_msg.z = beacon_pose.z;
        beacon_pose_msg.id = beacon_pose.id;
        msg.collection.push_back(beacon_pose_msg);
    }

    relative_beacon_publisher_->publish(msg);
}

void FalconManagerNode::publishRelativeAngle(const Sds::Falcon::RelativeAngleCollection& col) {
    falcon_pkg::msg::RelativeAngleCollection msg;

    for (const auto& angle : col) {
        falcon_pkg::msg::RelativeAngle angle_msg;
        angle_msg.x_angle = angle.x_angle;
        angle_msg.z_angle = angle.z_angle;
        angle_msg.id = angle.id;
        angle_msg.intensity = angle.intensity;
        angle_msg.width = angle.width;
        msg.collection.push_back(angle_msg);
    }

    relative_angle_publisher_->publish(msg);
}

void FalconManagerNode::publish6Dof(const Sds::Falcon::Pose6Dof& sd) {
    falcon_pkg::msg::Pose6Dof msg;
    msg.valid = sd.valid;
    msg.x = sd.x;
    msg.y = sd.y;
    msg.z = sd.z;
    msg.qw = sd.qw;
    msg.qx = sd.qx;
    msg.qy = sd.qy;
    msg.qz = sd.qz;
    msg.var_x = sd.var_x;
    msg.var_y = sd.var_y;
    msg.var_z = sd.var_z;
    msg.var_h = sd.var_h;
    msg.var_p = sd.var_p;
    msg.var_r = sd.var_r;
    
    sixdof_publisher_->publish(msg);
}

void FalconManagerNode::publishCurrentShutter(const Sds::Falcon::CurrentShutter& cs) {
    falcon_pkg::msg::CurrentShutter msg;
    msg.shutter_value = cs.shutter_value;
    
    current_shutter_publisher_->publish(msg);
}

void FalconManagerNode::publishFieldOfViewReport(const Sds::Falcon::FieldOfViewReport& fov) {
    falcon_pkg::msg::FieldOfViewReport msg;
    
    for (const auto& fovPair : fov.seenBeacons) {
        msg.keys.push_back(fovPair.first);
        msg.values.push_back(fovPair.second);
    }

    fov_report_publisher_->publish(msg);
}

void FalconManagerNode::publishDynamicMessage(const Sds::Falcon::DynamicMessage& dm) {
    falcon_pkg::msg::DynamicMessage msg;

    msg.valid = dm.valid;
    msg.beacon_id = dm.beacon_id;
    msg.message = dm.message.c_str();

    dynamic_message_publisher_->publish(msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FalconManagerNode>());
    rclcpp::shutdown();
    return 0;
}