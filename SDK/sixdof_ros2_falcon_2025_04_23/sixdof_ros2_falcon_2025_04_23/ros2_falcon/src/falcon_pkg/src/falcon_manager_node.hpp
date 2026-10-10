#include <rclcpp/rclcpp.hpp>
#include "SdsFalcon.h"

#include "falcon_pkg/srv/get_version.hpp"
#include "falcon_pkg/srv/get_firmware_version.hpp"
#include "falcon_pkg/msg/heartbeat.hpp"
#include "falcon_pkg/msg/status_message.hpp"
#include "falcon_pkg/srv/initialize_network.hpp"
#include "falcon_pkg/srv/initialize_network_advanced.hpp"
#include "falcon_pkg/srv/set_tracking_mode.hpp"

#include "falcon_pkg/srv/initialize_relative_beacon.hpp"
#include "falcon_pkg/msg/pose_relative_beacon.hpp"
#include "falcon_pkg/msg/pose_relative_beacon_collection.hpp"

#include "falcon_pkg/srv/initialize_relative_angle.hpp"
#include "falcon_pkg/msg/relative_angle.hpp"
#include "falcon_pkg/msg/relative_angle_collection.hpp"

#include "falcon_pkg/srv/initialize6_dof.hpp"
#include "falcon_pkg/msg/pose6_dof.hpp"
#include "falcon_pkg/msg/beacon.hpp"
#include "falcon_pkg/msg/gyro_offset.hpp"

#include "falcon_pkg/msg/current_shutter.hpp"
#include "falcon_pkg/msg/field_of_view_report.hpp"

#include "falcon_pkg/srv/swap_map.hpp"
#include "falcon_pkg/srv/get_gyro_calibration.hpp"

#include "falcon_pkg/msg/dynamic_message.hpp"

#include "falcon_pkg/srv/load_shutter_plugin.hpp"
#include "falcon_pkg/srv/load_default_shutter_control.hpp"


class FalconManagerNode : public rclcpp::Node {
public:
    FalconManagerNode();

private:
    std::unique_ptr<Sds::Falcon::FalconManager> falconManager_;
    Sds::Falcon::NetworkConfig networkConfig;
    Sds::Falcon::NetworkAdvancedConfig networkAdvancedConfig;
    Sds::Falcon::ConfigRelativeBeacon configRelB;
    Sds::Falcon::ConfigRelativeAngle configRelA;
    Sds::Falcon::Config6Dof config6Dof;

    rclcpp::Service<falcon_pkg::srv::GetVersion>::SharedPtr get_version_service_;
    rclcpp::Service<falcon_pkg::srv::GetFirmwareVersion>::SharedPtr get_firmware_version_service_;
    rclcpp::Service<falcon_pkg::srv::InitializeNetwork>::SharedPtr initialize_network_service_;
    rclcpp::Service<falcon_pkg::srv::InitializeNetworkAdvanced>::SharedPtr initialize_network_advanced_service_;
    rclcpp::Service<falcon_pkg::srv::SetTrackingMode>::SharedPtr set_tracking_mode_service_;
    rclcpp::Service<falcon_pkg::srv::InitializeRelativeBeacon>::SharedPtr initialize_relative_beacon_service_;
    rclcpp::Service<falcon_pkg::srv::InitializeRelativeAngle>::SharedPtr initialize_relative_angle_service_;
    rclcpp::Service<falcon_pkg::srv::Initialize6Dof>::SharedPtr initialize_6dof_service_;
    rclcpp::Service<falcon_pkg::srv::SwapMap>::SharedPtr swap_map_service_;
    rclcpp::Service<falcon_pkg::srv::GetGyroCalibration>::SharedPtr get_gyro_calibration_service_;
    rclcpp::Service<falcon_pkg::srv::LoadShutterPlugin>::SharedPtr load_shutter_plugin_service_;
    rclcpp::Service<falcon_pkg::srv::LoadDefaultShutterControl>::SharedPtr load_default_shutter_control_service_;
    
    rclcpp::Publisher<falcon_pkg::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
    rclcpp::Publisher<falcon_pkg::msg::StatusMessage>::SharedPtr status_message_publisher_;
    rclcpp::Publisher<falcon_pkg::msg::PoseRelativeBeaconCollection>::SharedPtr relative_beacon_publisher_;
    rclcpp::Publisher<falcon_pkg::msg::RelativeAngleCollection>::SharedPtr relative_angle_publisher_;
    rclcpp::Publisher<falcon_pkg::msg::Pose6Dof>::SharedPtr sixdof_publisher_;
    rclcpp::Publisher<falcon_pkg::msg::CurrentShutter>::SharedPtr current_shutter_publisher_;
    rclcpp::Publisher<falcon_pkg::msg::FieldOfViewReport>::SharedPtr fov_report_publisher_;
    rclcpp::Publisher<falcon_pkg::msg::DynamicMessage>::SharedPtr dynamic_message_publisher_;

    void handleGetVersion(const std::shared_ptr<falcon_pkg::srv::GetVersion::Request> request,
                          std::shared_ptr<falcon_pkg::srv::GetVersion::Response> response);

    void handleGetFirmwareVersion(const std::shared_ptr<falcon_pkg::srv::GetFirmwareVersion::Request> request,
                          std::shared_ptr<falcon_pkg::srv::GetFirmwareVersion::Response> response);

    void handleInitializeNetwork(const std::shared_ptr<falcon_pkg::srv::InitializeNetwork::Request> request,
                          std::shared_ptr<falcon_pkg::srv::InitializeNetwork::Response> response);
    
    void handleInitializeNetworkAdvanced(const std::shared_ptr<falcon_pkg::srv::InitializeNetworkAdvanced::Request> request,
                          std::shared_ptr<falcon_pkg::srv::InitializeNetworkAdvanced::Response> response);

    void handleSetTrackingMode(const std::shared_ptr<falcon_pkg::srv::SetTrackingMode::Request> request,
                          std::shared_ptr<falcon_pkg::srv::SetTrackingMode::Response> response);

    void handleInitializeRelativeBeacon(const std::shared_ptr<falcon_pkg::srv::InitializeRelativeBeacon::Request> request,
                          std::shared_ptr<falcon_pkg::srv::InitializeRelativeBeacon::Response> response);

    void handleInitializeRelativeAngle(const std::shared_ptr<falcon_pkg::srv::InitializeRelativeAngle::Request> request,
                          std::shared_ptr<falcon_pkg::srv::InitializeRelativeAngle::Response> response);

    void handleInitialize6Dof(const std::shared_ptr<falcon_pkg::srv::Initialize6Dof::Request> request,
                          std::shared_ptr<falcon_pkg::srv::Initialize6Dof::Response> response);

    void handleSwapMap(const std::shared_ptr<falcon_pkg::srv::SwapMap::Request> request,
                          std::shared_ptr<falcon_pkg::srv::SwapMap::Response> response);

    void handleGetGyroCalibration(const std::shared_ptr<falcon_pkg::srv::GetGyroCalibration::Request> request,
                          std::shared_ptr<falcon_pkg::srv::GetGyroCalibration::Response> response);

    void handleLoadShutterPlugin(const std::shared_ptr<falcon_pkg::srv::LoadShutterPlugin::Request> request,
                          std::shared_ptr<falcon_pkg::srv::LoadShutterPlugin::Response> response);

    void handleLoadDefaultShutterControl(const std::shared_ptr<falcon_pkg::srv::LoadDefaultShutterControl::Request> request,
                          std::shared_ptr<falcon_pkg::srv::LoadDefaultShutterControl::Response> response);
    
    void publishHeartbeat(const Sds::Falcon::Heartbeat& heartbeat);

    void publishStatusMessage(const Sds::Falcon::StatusMessage& statusMessage);

    void publishRelativeBeacon(const Sds::Falcon::PoseRelativeBeaconCollection& col);

    void publishRelativeAngle(const Sds::Falcon::RelativeAngleCollection& col);

    void publish6Dof(const Sds::Falcon::Pose6Dof& sd);

    void publishCurrentShutter(const Sds::Falcon::CurrentShutter& cs);

    void publishFieldOfViewReport(const Sds::Falcon::FieldOfViewReport& fov);

    void publishDynamicMessage(const Sds::Falcon::DynamicMessage& dm);

    Sds::Falcon::TrackingMode getTrackingMode(const std::string& modeStr);
    
    Sds::Falcon::ShutterSettings getShutterSettings(uint8_t minVal, uint8_t maxVal);
};