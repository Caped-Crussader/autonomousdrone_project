//Copyright © 2017-2024 Six Degrees Space Ltd.  All rights reserved.
//Proprietary and Confidential.  Unauthorized use, disclosure or reproduction is strictly prohibited.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/operators.h>
#include <pybind11/functional.h>
namespace py = pybind11;

#include "SdsFalcon.h"
#include "SdsShutterControl.h"

namespace Sds {
    namespace Falcon {

        PYBIND11_MODULE(pySdsFalcon, g) {

            py::class_<Version>(g, "Version")
                .def("getMajor", &Version::getMajor)
                .def("getMinor", &Version::getMinor)
                .def("getPatch", &Version::getPatch)
                .def("isEqual", &Version::isEqual)
                .def("isAtLeast", &Version::isAtLeast)
                .def("getString", &Version::getString);

            g.def("getVersion", [](){ return getVersion(); });

            py::class_<NetworkConfig>(g, "NetworkConfig")
                .def(py::init<>())
                .def_readwrite("sensor_id", &NetworkConfig::sensor_id)
                .def_readwrite("sensor_network_name", &NetworkConfig::sensor_network_name)
                .def_readwrite("sensor_ip", &NetworkConfig::sensor_ip)
                .def_readwrite("udp_read_port", &NetworkConfig::udp_read_port);

            py::class_<Beacon>(g, "Beacon")
                .def(py::init<>())
                .def_readwrite("x", &Beacon::x)
                .def_readwrite("y", &Beacon::y)
                .def_readwrite("z", &Beacon::z)
                .def_readwrite("id", &Beacon::id);

            py::class_<Pose6Dof>(g, "Pose6Dof")
                .def_readonly("valid", &Pose6Dof::valid)
                .def_readonly("x", &Pose6Dof::x)
                .def_readonly("y", &Pose6Dof::y)
                .def_readonly("z", &Pose6Dof::z)
                .def_readonly("qw", &Pose6Dof::qw)
                .def_readonly("qx", &Pose6Dof::qx)
                .def_readonly("qy", &Pose6Dof::qy)
                .def_readonly("qz", &Pose6Dof::qz)
                .def_readonly("var_x", &Pose6Dof::var_x)
                .def_readonly("var_y", &Pose6Dof::var_y)
                .def_readonly("var_z", &Pose6Dof::var_z)
                .def_readonly("var_h", &Pose6Dof::var_h)
                .def_readonly("var_p", &Pose6Dof::var_p)
                .def_readonly("var_r", &Pose6Dof::var_r);

            py::class_<RelativeAngle>(g, "RelativeAngle")
                .def_readonly("x_angle", &RelativeAngle::x_angle)
                .def_readonly("z_angle", &RelativeAngle::z_angle)
                .def_readonly("id", &RelativeAngle::id)
                .def_readonly("intensity", &RelativeAngle::intensity)
                .def_readonly("width", &RelativeAngle::width);

            py::bind_vector<RelativeAngleCollection>(g, "RelativeAngleCollection");
            
            py::enum_<Severity>(g, "Severity")
                .value("Exception", Severity::Exception)
                .value("Warning", Severity::Warning)
                .value("Informative", Severity::Informative);

            py::enum_<CallingLayer>(g, "CallingLayer")
                .value("SdsFalcon", CallingLayer::SdsFalcon)
                .value("SdsCommLib", CallingLayer::SdsCommLib)
                .value("NetworkConnection", CallingLayer::NetworkConnection)
                .value("InternalAlgorithm", CallingLayer::InternalAlgorithm);

            py::enum_<FalconEventCode>(g, "FalconEventCode")
                .value("TrackingNotInitalized", FalconEventCode::TrackingNotInitalized)
                .value("NetworkNotInitalized", FalconEventCode::NetworkNotInitalized)
                .value("GyroCalibrationRejected", FalconEventCode::GyroCalibrationRejected)
                .value("CommunicationLoopNotClosed", FalconEventCode::CommunicationLoopNotClosed)
                .value("DidNotReceiveFirmwareInfo", FalconEventCode::DidNotReceiveFirmwareInfo)
                .value("SensorFirmwareIsNotLatest", FalconEventCode::SensorFirmwareIsNotLatest)
                .value("SensorFirmwareIsNotCompatible", FalconEventCode::SensorFirmwareIsNotCompatible)
                .value("DidNotReceive6DofMap", FalconEventCode::DidNotReceive6DofMap)
                .value("ConfigHasBothNetworkAndIp", FalconEventCode::ConfigHasBothNetworkAndIp)
                .value("PlaybackReachedEndOfFile", FalconEventCode::PlaybackReachedEndOfFile);

            py::class_<StatusMessage>(g, "StatusMessage")
                .def_readonly("severity", &StatusMessage::severity)
                .def_readonly("layer", &StatusMessage::layer)
                .def_readonly("event_code", &StatusMessage::event_code)
                .def_readonly("message", &StatusMessage::message);

            py::class_<DynamicMessage>(g, "DynamicMessage")
                .def_readonly("valid", &DynamicMessage::valid)
                .def_readonly("beacon_id", &DynamicMessage::beacon_id)
                .def_readonly("message", &DynamicMessage::message);

            py::enum_<TrackingState>(g, "TrackingState")
                .value("Initializing", TrackingState::Initializing)
                .value("Running", TrackingState::Running)
                .value("Error", TrackingState::Error);

            g.def("to_string", py::overload_cast<TrackingState>(&to_string));

            py::class_<Heartbeat>(g, "Heartbeat")
                .def_readonly("sensor_communication_ok", &Heartbeat::sensor_communication_ok)
                .def_readonly("current_tracking_state", &Heartbeat::current_tracking_state);

            py::class_<CurrentShutter>(g, "CurrentShutter")
                .def_readonly("shutter_value", &CurrentShutter::shutter_value);

            py::class_<GyroOffset>(g, "GyroOffset")
                .def_readonly("sensorId", &GyroOffset::sensorId)
                .def_readonly("datetime", &GyroOffset::datetime)
                .def_readonly("temperature", &GyroOffset::temperature)
                .def_readonly("gyro_x", &GyroOffset::gyro_x)
                .def_readonly("gyro_y", &GyroOffset::gyro_y)
                .def_readonly("gyro_z", &GyroOffset::gyro_z)
                .def_readonly("standard_dev", &GyroOffset::standard_dev);

            py::class_<FieldOfViewReport>(g, "FieldOfViewReport")
                .def_readonly("seenBeacons", &FieldOfViewReport::seenBeacons);
            
            py::enum_<DataPlaybackMode>(g, "DataPlaybackMode")
                .value("NONE", DataPlaybackMode::NONE)
                .value("RECORD", DataPlaybackMode::RECORD)
                .value("PLAYBACK", DataPlaybackMode::PLAYBACK);

            py::class_<DataPlaybackSettings>(g, "DataPlaybackSettings")
                .def(py::init<>())
                .def_readwrite("mode", &DataPlaybackSettings::mode)
                .def_readwrite("path", &DataPlaybackSettings::path);

            py::class_<Config>(g, "Config")
                .def(py::init<>())
                .def_readwrite("networkConfig", &Config::networkConfig)
                .def_readwrite("map", &Config::map)
                .def_readwrite("gyroCalibration", &Config::gyroCalibration)
                .def_readwrite("skip_gyro_calibration", &Config::skip_gyro_calibration)
                .def_readwrite("no_gyro_mode", &Config::no_gyro_mode);

            g.def("saveConfig", &saveConfig);

            g.def("loadConfig", &loadConfig);

            g.def("loadMap", &loadMap);

            py::class_<ConfigGyroCalibration>(g, "ConfigGyroCalibration")
                .def(py::init<>())
                .def_readwrite("duration_seconds", &ConfigGyroCalibration::duration_seconds);

            py::class_<FalconManager>(g, "FalconManager")
                .def("initialize", py::overload_cast<const Config&>(&FalconManager::initialize), py::arg("config"))
                .def("initialize", py::overload_cast<const Config&, const DataPlaybackSettings&>(&FalconManager::initialize), py::arg("config"), py::arg("playback_settings"))
                .def("start", &FalconManager::start)
                .def("stop", &FalconManager::stop)
                .def("getFirmwareVersion", &FalconManager::getFirmwareVersion)
                .def("getGyroCalibration", &FalconManager::getGyroCalibration)
                .def("getPose6Dof", &FalconManager::getPose6Dof)
                .def("getRelativeAngleCollection", &FalconManager::getRelativeAngleCollection)
                .def("getStatusMessage", &FalconManager::getStatusMessage)
                .def("getDynamicMessage", &FalconManager::getDynamicMessage)
                .def("getFieldOfViewReport", &FalconManager::getFieldOfViewReport)
                .def("getCurrentShutter", &FalconManager::getCurrentShutter)
                .def("registerPose6DofCallback", &FalconManager::registerPose6DofCallback)
                .def("registerRelativeAngleCallback", &FalconManager::registerRelativeAngleCallback)
                .def("registerMessageCallback", &FalconManager::registerMessageCallback)
                .def("registerDynamicMessageCallback", &FalconManager::registerDynamicMessageCallback)
                .def("registerFieldOfViewReportCallback", &FalconManager::registerFieldOfViewReportCallback)
                .def("registerHeartbeatCallback", &FalconManager::registerHeartbeatCallback)
                .def("registerCurrentShutterCallback", &FalconManager::registerCurrentShutterCallback)
                .def("removePose6DofCallback", &FalconManager::removePose6DofCallback)
                .def("removeRelativeAngleCallback", &FalconManager::removeRelativeAngleCallback)
                .def("removeMessageCallback", &FalconManager::removeMessageCallback)
                .def("removeDynamicMessageCallback", &FalconManager::removeDynamicMessageCallback)
                .def("removeFieldOfViewCallback", &FalconManager::removeFieldOfViewCallback)
                .def("removeHeartbeatCallback", &FalconManager::removeHeartbeatCallback)
                .def("removeCurrentShutterCallback", &FalconManager::removeCurrentShutterCallback)
                .def("swapMap", &FalconManager::swapMap)
                .def("setShutterControl", &FalconManager::setShutterControl)
                .def("loadShutterPlugin", &FalconManager::loadShutterPlugin)
                .def("loadDefaultShutterControl", &FalconManager::loadDefaultShutterControl);

            g.def("createFalconManager", &createFalconManager);

        }

    }
}
