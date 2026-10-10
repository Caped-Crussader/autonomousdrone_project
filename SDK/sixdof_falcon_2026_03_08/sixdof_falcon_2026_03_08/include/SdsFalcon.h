//Copyright © 2017-2026 Six Degrees Space Ltd.  All rights reserved.
//Proprietary and Confidential.  Unauthorized use, disclosure or reproduction is strictly prohibited.

/**
 * @file
 * @brief Header file containing declarations for the Sixdof Falcon library.
 *
 * This file contains the declarations for classes and data structures related to the Sixdof Falcon library.
 * It provides functionality for managing the Sixdof sensor and the Sixdof Falcon tracking system.
 */

#pragma once 
#include <stdint.h>
#include <map>
#include <vector>
#include <string>
#include <memory>
#include <functional>

#include "SdsShutterControl.h"

#ifdef _WIN32
    // Windows platform
    #ifdef SDS_EXPORTS
        #define SDS_FALCON_API __declspec(dllexport)
    #else
        #define SDS_FALCON_API __declspec(dllimport)
    #endif
#else
    // Non-Windows platform
    #define SDS_FALCON_API __attribute__((visibility("default")))
#endif

#if defined(__GNUC__) || defined(__clang__)
#define DEPRECATED(msg) __attribute__((deprecated(msg)))
#elif defined(_MSC_VER)
#define DEPRECATED(msg) __declspec(deprecated(msg))
#else
#define DEPRECATED(msg)
#endif

/**
 * @mainpage Sixdof Falcon Documentation
 * 
 * @image html Falcon_Logo.png "" width=400px
 * @image latex Falcon_Logo.png "" width=300px
 *
 * @section intro_sec Introduction
 * Sixdof Space has created an optical tracking solution offering tracking with cm level accuracy both indoors and outdoors in direct sunlight. 
 * Our patented technology leverages infrared lighting to serve as location beacons.
 * The Falcon SDK contains algorithms that will report the 6DOF positional data of the sensor relative to the beacons, 400 times a second.
 * By connecting the sensor to your drone, robot, or VR headset, the Sixdof Falcon tracking system can provide full 6DOF tracking to your platform.
 * 
 * SdsFalcon is a shared library that enables users to interface with the Sixdof tracking technology.
 * Drone guidance is the main application of this library, however other applications that require a programmatic interface to the Sixdof technology can use this library as well. 
 *
 * The SdsFalcon shared library is available on the following platforms:
 * - Windows 
 * - Linux Ubuntu
 * - Linux Raspberry Pi OS (32 bit)
 * - Linux Raspberry Pi OS (64 bit)
 * - Nvidia Jetson 
 * 
 * If you want to use the SdsFalcon on another platform, please contact the Sixdof team.
 * 
 * @section network_setup Network Setup
 * To connect the sensor to the host computer, first connect the sensor's USB to a 5V power source. Next connect the RJ45 port to a network port on the host computer, or via a USB to Ethernet dongle.
 * On Linux based computers, you must set up the network before operation of the sensor. Additionally, to achieve best performance on Linux platforms, Wifi should be turned off during operation. 
 * The net-tools package is required to connect to the sensor. You can test if you have the net-tools package by typing "ifconfig" into the command line, if the response says "command not found" please install the net-tools package via the command: "sudo apt install net-tools". 
 * The following instructions are for Ubuntu based distributions, the Raspberry Pi 3b instructions are shown below.
 * First navigate to the Network Settings and ensure the "Wired" connection is set to "ON":
 * @image html network_ubuntu01.png "" width=400px
 * @image latex network_ubuntu01.png "" width=300px
 * 
 * If you are using a USB dongle there will be another row labeled "USB Ethernet", set that to "ON". Next click the gear button to go to the advanced settings:
 * @image html network_ubuntu02.png "" width=400px
 * @image latex network_ubuntu02.png "" width=300px
 * 
 * In the "Details" tab, deselect the checkbox that says "Connect automatically". Next navigate to the IPv4 tab and set the "IPv4 Method" to "Manual" and set the "Address" and "Netmask". 
 * The Address can be any IP address that you choose, however it should not clash with any other networks on the host computer. To avoid IP address clashes as much as possible you can set the Netmask to 255.255.255.0.
 * Additionally, set the "DNS Automatic" to "OFF" as shown below:
 * @image html network_ubuntu03.png "" width=400px
 * @image latex network_ubuntu03.png "" width=300px
 * 
 * Finally, navigate to the IPv6 tab and for the "IPv6 Method" select "Disable":
 * @image html network_ubuntu04.png "" width=400px
 * @image latex network_ubuntu04.png "" width=300px
 * 
 * On Raspberry Pi 3b, navigate to the "Network Preferences" menu, next to "Configure" leave the first drop down on the "interface" option, and on the second drop down select the network that you are connected to, typically "eth0". If you are using a USB dongle the network may have a different name.
 * Deselect the options that says "Automatically configure empty options", and select "Disable IPv6". Next fill in an IP Address that you selected and put "/24" at the end to indicate that you want a submask of 255.255.255.0.
 * An example is shown below:
 * @image html network_rpi.png "" width=300px
 * @image latex network_rpi.png "" width=250px
 * 
 * @section coordinate_sys_sec Coordinate Systems
 * Before using the Sixdof tracking system it is important to understand the coordinate systems involved. The sensor coordinate system is defined according to the image below:
 * @image html kipa_with_axes.png "Sensor coordinate system" width=300px
 * @image latex kipa_with_axes.png "Sensor coordinate system" width=250px
 * 
 * Where the positive x-axis points to the right of the sensor, the positive y-axis point out from the sensor and the positive z-axis completes the right handed system. 
 * The field of view can be approximated by a +/- 60 degree vision cone around the y-axis.
 * 
 * In many applications, the user is interested in the location of the host platform (drone, or robot), and not just the location of the sensor.
 * In these applications, it is the user's responsibility to do the rigid body transformation from the sensor to the host platform, based on how the sensor is mounted on the platform. 
 * 
 * @section tracking_data_sec Tracking Data
 * There are two representations of tracking data available simultaneously:
 * - 6DOF: the full six degrees of freedom (position and orientation) relative to the map.
 * - Relative Angle: the angles of each beacon relative to the sensor, this is intended for applications where the sensor is far from the beacons.
 * 
 * In addition to tracking, a method is provided to enable the user to do gyroscope calibration. 
 * During gyroscope calibration, the sensor must be still. In many applications, it is desirable to initialize the system while already in motion.
 * Therefore, gyroscope calibration can be done in advance to the mission and the calibrated values can be fed in at run time. 
 * Alternatively, the gyro calibration step can be ignored, this is sufficient in most cases.
 *
 * @section sixdof_sec 6DOF Data
 * @image html 6DOF_Light.png "" width=400px
 * @image latex 6DOF_Light.png "" width=250px
 * The 6DOF data presents the position of the sensor tracked relative to the map. The map is made up of multiple IR beacons that flash unique ids, multiple beacons are driven by a single beacon driver board.
 * The user must place these beacons in a rigid formation and provide the X, Y, Z location of each beacon. The unique id of each beacon is provided on each LED individually.
 * The user can define the coordinate system of the map in any way they choose, however it must be a right handed coordinate system.
 * When designing a map, it is important to ensure the lights are not co-linear. Keep in mind the desired area of operation, try to design the map so that there will always be at least 3 beacons in the field of view of the sensor.
 *  
 * @section rel_angle_sec Relative Angle Data
 * @image html RelA_Light.png "" width=400px
 * @image latex RelA_Light.png "" width=250px
 * The Relative Angle data presents the angle of a beacon tracked relative to the coordinate system of the sensor.
 * For example, this data can be used to guide a drone from a high elevation all the way down to a landing pad. This data does not provide any orientation information of the senor.
 * Additionally, due to the far distance of the beacon, the baseline on the sensor is insufficient to calculate the distance directly.
 * The intensity of the beacon and the peak width in pixels are reported in this data in order to get a rough estimate of the distance of the beacon.   
 * 
 * @section change_log_sec Change Log
 *    Version    | Date       | Notes |
 * ------------- | ---------- | ----- |
 * 2.0.1         | 08/03/2026 | Bug fix to Relative Angles mode. |
 * 2.0.0         | 08/01/2026 | 6DOF and Relative Angles run simultaneously. Removed Relative Beacon. Added Playback. Added DevFeatures. Removed ShutterSettings from the config. |
 * 1.7.4         | 19/10/2025 | Algorithm changes including: kipa-pro support, localization uses P3P algorithm, changed gyro noise parameters, better velocity estimation. |
 * 1.7.3         | N/A        | Fix for carnea member in ShutterControl, added getShutterSettingsDefault so that getShutterSettingsAuto can refer to pure auto shutter mode. |
 * 1.7.2         | 18/06/2025 | Fixes for static ip connection. |
 * 1.7.1         | 12/05/2025 | Better peak height estimation. |
 * 1.7.0         | 24/11/2024 | Added Dynamic Messages, changed beacon Ids to uint32_t, and added support for shutter plugins |
 * 1.6.0         | 21/07/2024 | Re-branded to Sixdof Falcon | 
 * 1.5.5         | 28/05/2024 | Added more elaborate check for the right firmware version, better network debugging tools, bug fix for Raspberry Pi platforms. |
 * 1.5.3         | 29/02/2024 | Added NetworkAdvancedConfig, getFirmwareVersion, skip_gyro_calibration is now defaulted to true |
 * 1.5.2         | 20/02/2024 | Added expected_beacon_ids to ConfigRelativeBeacon and ConfigRelativeAngle, also changed shutter backed, deprecated setShutter function |
 * 1.5.1         | 01/02/2024 | Added skip_gyro_calibration flag to Config6Dof |
 * 1.5.0         | 20/12/2023 | Added setShutter feature, current shutter callback, put shutter_settings into each of the config objects |
 * 1.4.1         | 27/11/2023 | Added swapMap feature |
 * 1.4.0         | 05/11/2023 | Added Relative Angle feature |
 * 1.3.0         | 17/10/2023 | Added health monitoring feature in the form of Heartbeat callbacks |
 * 1.2.1         | 06/09/2023 | Exposed "No Gyro" mode as a parameter in Config6Dof. Made changes to support multiple sensors each with its own FalconManager |
 * 1.2.0         | 22/08/2023 | Changed API so that PoseRelativeBeacons that happen at the same time are bundled together |
 * 1.1.2         | 14/08/2023 | Updated RelativeBeacon logic, parameter rel_lights_cooldown_ms is no longer needed, instead we have the parameter field_of_view_cutoff_deg |
 * 1.1.1         | 14/08/2023 | Statically link GCC libraries, add MSVC build to the release |
 * 1.1.0-beta    | 24/07/2023 | Changed header, included gyro calibration, field of view report features and data logging for replay |
 * 1.0.0-beta    | 01/05/2023 | inital beta version |
 * 
 * @section troubleshooting Trouble Shooting Guide
 * @subsection missing_libraries Missing Libraries
 * Make sure to always copy the shared library (SdsFalcon.dll on Windows and SdsFalcon.so on Linux) that are provided in the release folder for your platform into the the directory with your program. 
 * Alternatively, you can statically link the shared library to your program. Also on Windows, you have the option of adding the shared library to your system PATH. 
 * 
 * @subsection ping_failed Ping Failed
 * Errors relating to ping failing mean that the host computer was unable to connect to the Sixdof sensor.
 * Try the following steps:
 *  -# Ensure the Sixdof sensor is connected to power and the RJ45 ethernet cable is connected to your computer.
 *  -# Look at the red indicator lights that are exposed on the top of the senor casing, the lights should be flashing. If the lights are constant, please contact the Sixdof team.
 *  -# Ensure your NetworkConfig parameters are correct, this is the sensor board number and the network name that the sensor is connected to.
 *  -# Your computer's ARP table is used to connect to the sensor, try clearing the ARP table and trying again. The ARP table can be cleared via a command console with administrator privileges, use the command "arp -d". 
 * 
 * @subsection firewall Firewall
 * If SdsFalcon is reporting errors related to communication issues, it is recommended to turn off your firewall.
 * 
 * @subsection underprocessing Underprocessing
 * If you are getting the "Warning" status message that says "Not receiving enough buffers from board" that means that your program is not getting enough CPU time. 
 * On Windows systems this is usually due to a spontaneous virus scan, you can temporarily turn off virus scans from in your "Windows Security" settings.
 */

namespace Sds {
    namespace Falcon {
    
        /**
         * @class Version
         * @brief A class representing the version of either the shared library, or the sensor firmware.
         *
         * This class provides utility functions around a system version (major, minor, patch), either the version of the shared library, or the sensor firmware version.
         */
        class SDS_FALCON_API Version {
        public:
            Version(uint8_t major, uint8_t minor, uint8_t patch);

            /**
             * @brief Get the major component of the version.
             * @return The major component of the version as a uint8_t.
             */
            uint8_t getMajor() const;

            /**
             * @brief Get the minor component of the version.
             * @return The minor component of the version as a uint8_t.
             */
            uint8_t getMinor() const;

            /**
             * @brief Get the patch component of the version.
             * @return The patch component of the version as a uint8_t.
             */
            uint8_t getPatch() const;

            /**
             * @brief Check if the version is equal to the specified components.
             *
             * @param major The major component to compare.
             * @param minor The minor component to compare.
             * @param patch The patch component to compare.
             * @return True if the version is equal to the specified components, false otherwise.
             */
            bool isEqual(uint8_t major, uint8_t minor, uint8_t patch) const;

            /**
             * @brief Check if the version is at least the specified version.
             *
             * @param major The major component to compare.
             * @param minor The minor component to compare.
             * @param patch The patch component to compare.
             * @return True if the version is at least the specified version, false otherwise.
             */
            bool isAtLeast(uint8_t major, uint8_t minor, uint8_t patch) const;

            /**
             * @brief Get a string representation of the version.
             * 
             * Get a string representation of the version.
             * @return A string representing the version.
             */
            std::string getString() const;

        private:
            uint8_t _major, _minor, _patch;
        };

        /**
         * @brief Get the version of the SdsFalcon shared library.
         * @return The Version object representing the library version.
         */
        SDS_FALCON_API Version getVersion();

        /**
         * @struct NetworkConfig
         * @brief A structure representing network configuration for the Sixdof sensor.
         *
         * This structure contains the necessary information in order to connect to a Sixdof sensor.
         * 
         * The sensor_ip field is for cases where the user will add the entry to the ARP table themselves.
         * Typically this is used on system where the basic network configuration will fail, for example in situations where the 
         * computer operating system is not in English.
         */
        struct SDS_FALCON_API NetworkConfig {
            uint16_t sensor_id;                      ///< sensor id.
            std::string sensor_network_name { "" };  ///< name of the network the sensor is connected to.
            std::string sensor_ip { "" };            ///< ip address of the sensor (optional). Note, this is for advanced configuration only, and should generally be left to the default value.
            uint16_t udp_read_port { 0 };            ///< udp port to read from the sensor, to be auto-assigned a port leave as 0. (Optional)
        };

        /**
         * @struct Beacon
         * @brief A structure representing a beacon with 3D coordinates and an id.
         *
         * This structure contains the 3D coordinates (x, y, z) of a beacon in meters and its unique id.
         */
        struct SDS_FALCON_API Beacon {
            double x;     ///< x-coordinate in meters.
            double y;     ///< y-coordinate in meters.
            double z;     ///< z-coordinate in meters.
            uint32_t id;  ///< unique id.
        };

        /**
         * @struct Pose6Dof
         * @brief A structure representing a 6DOF pose.
         *
         * This structure contains 6DOF (6 degree of freedom) pose information, specifically the pose of the Sixdof sensor with respect to the map. 
         * The pose is represented by position (x, y, z) in meters, and orientation as a quaternion (qx, qw, qy, qz).
         * Pose accuracy is represented as variance of the position (var_x, var_y, var_z) in meters squared, and the variance of the orientation (var_h, var_p, var_r) in radians squared.
         * 
         * This is the return type for the 6DOF tracking data.
         */
        struct SDS_FALCON_API Pose6Dof {
            bool valid { false };   ///< flag indicating if the pose is valid. This can be False when calling the getPose6Dof function before a valid pose is obtained.
            double x;               ///< x-coordinate in meters.
            double y;               ///< y-coordinate in meters.
            double z;               ///< z-coordinate in meters.
            double qw;              ///< quaternion w component.
            double qx;              ///< quaternion x component.
            double qy;              ///< quaternion y component.
            double qz;              ///< quaternion x component.
            float var_x, var_y, var_z; // m^2
            float var_h, var_p, var_r; // rad^2
        };

        /**
         * @struct RelativeAngle
         * @brief A structure representing the relative angle of a beacon.
         *
         * This structure contains the angle of a single beacon with respect to the sensor.
         * Where x_angle is the angular offset along the x-axis, and z_angle is the angular offset along the z-axis.
         */
        struct SDS_FALCON_API RelativeAngle {
            double x_angle;           ///< angular offset in the x-axis in radians.
            double z_angle;           ///< angular offset in the z-axis in radians.
            uint32_t id;              ///< unique id of the beacon.
            double intensity;         ///< light intensity of the beacon, this can be used to roughly indicate distance.
            double width;             ///< width of the detected peak in pixels, also useful for indicating distance.
        };

        /**
         * @typedef RelativeAngleCollection
         * @brief A structure representing a collection of RelativeAngle.
         *
         * This structure contains multiple RelativeAngle estimates. 
         * All RelativeAngle estimates were calculated at the same and therefore bundled together.
         * 
         * This is the return type for RelativeAngle tracking data.
         */
        typedef std::vector<RelativeAngle> RelativeAngleCollection;

        /**
         * @enum Severity
         * @brief An enum representing the severity level of a status message.
         *
         * This enumeration defines severity levels for status messages.
         */
        enum class Severity {
            Exception = 1, ///< Exception indicates that there was a error in operation.
            Warning,       ///< Warning indicates that something went wrong but it is not critical.
            Informative,   ///< Informative indicates general debugging information, these can be ignored.
        };

        /**
         * @enum CallingLayer
         * @brief An enum representing the calling layer for a status message.
         *
         * This enum defines the calling layers for status messages.
         * Each layer indicates the source of a particular message.
         */
        enum class CallingLayer {
            SdsFalcon = 1,           ///< Message from the top layer of the library. Generally indicates the library was used incorrectly.
            SdsCommLib,              ///< Message from the UDP communication library, this is used to communicate with the sensor.
            NetworkConnection,       ///< Message from the network connection stage.
            InternalAlgorithm,       ///< Message from the internal Falcon algorithm.
        };

        /**
         * @enum FalconEventCode
         * @brief An enum representing event codes specific to the SdsFalcon calling layer.
         *
         * This enum defines event codes that are specific to the SdsFalcon calling layer.
         */
        enum class FalconEventCode {
            TrackingNotInitalized,         ///< indicates tracking was started before initialization.
            NetworkNotInitalized,          ///< indicates that the initializeNetwork function was not called.
            GyroCalibrationRejected,       ///< indicates gyro calibration values were rejected.
            CommunicationLoopNotClosed,    ///< communication loop is not closed, check your firewall.
            DidNotReceiveFirmwareInfo,     ///< did not receive firmware information from the sensor.
            SensorFirmwareIsNotLatest,     ///< sensor firmware is not latest version, it is not guaranteed that all SDK features will work fully.
            SensorFirmwareIsNotCompatible, ///< sensor firmware is not compatible with current SDK, please contact Sixdof for an upgrade. 
            DidNotReceive6DofMap,          ///< did not receive beacon map information in the Config.
            ConfigHasBothNetworkAndIp,     ///< received both the sensor network name and the sensor ip in the Config.
            PlaybackReachedEndOfFile,      ///< data playback feature reached the end of file.
        };

        /**
         * @struct StatusMessage
         * @brief A structure representing a status message.
         *
         * This structure contains information about a status message, including its severity,
         * calling layer, event code, and the content of the message.
         */
        struct SDS_FALCON_API StatusMessage {
            Severity severity;   ///< severity level of the status message.
            CallingLayer layer;  ///< calling layer for the status message.
            uint8_t event_code;  ///< event code of the status message.
            std::string message; ///< verbose status message.
        };

        /**
         * @struct DynamicMessage
         * @brief A structure representing a dynamic message.
         * 
         * This structure contains a "Dynamic" message. 
         * Dynamic messages are messages that are transmitted to the Sixdof sensor optically, via one of the Sixdof smart driver boards.
         */
        struct SDS_FALCON_API DynamicMessage {
            bool valid { false };
            uint32_t beacon_id;  ///< unique id of the source beacon.
            std::string message; ///< message that was transmitted optically.
        };

        /**
         * @enum TrackingState
         * @brief An enum representing different states of the tracking.
         */
        enum class TrackingState {
            Initializing, ///< tracking is currently initializing.
            Running,      ///< tracking is currently running.
            Error,        ///< tracking is currently in the error state.
        };

        /**
         * @brief Convert a TrackingState enum value to a string.
         *
         * @param state TrackingState enum value.
         * @return string representation of the TrackingState.
         */
        SDS_FALCON_API std::string to_string(TrackingState state);

        /**
         * @struct Heartbeat
         * @brief A structure representing health data of the SDK.
         *
         * This structure is output via a callback every second. This enables the user to quickly identify that the SDK is still running.
         * Additionally, the Heartbeat structure contains information regarding the state of communications with the sensor and the tracking algorithm.
         */
        struct SDS_FALCON_API Heartbeat {
            bool sensor_communication_ok { false };                               ///< status of communication with the sensor.
            TrackingState current_tracking_state { TrackingState::Initializing }; ///< state of the tracking.
        };

        /**
         * @struct CurrentShutter
         * @brief A structure representing the current shutter value.
         *
         * This structure is output via a callback, and indicates the current shutter value.
         * Shutter values are between 0 and 16, where 0 is the shortest shutter and 16 is the longest.
         */
        struct SDS_FALCON_API CurrentShutter {
            uint8_t shutter_value; ///< current shutter value
        };

        /**
         * @struct GyroOffset
         * @brief A structure representing a single gyro calibration offset for a specific sensor.
         *
         * This structure contains information about gyro calibration offset for a specific sensor.
         */
        struct SDS_FALCON_API GyroOffset {
            uint16_t sensorId { 0 };     ///< sensor id.
            std::string datetime { "" }; ///< date time in Y-m-d H:M:S format.
            int8_t temperature { 0 };    ///< temperature in degrees. This is the internal temperature of board, not the temperature of the external environment.
            int16_t gyro_x { 0 };        ///< bias in the x-coordinate of the IMU. Units are in gyro units.
            int16_t gyro_y { 0 };        ///< bias in the y-coordinate of the IMU. Units are in gyro units.
            int16_t gyro_z { 0 };        ///< bias in the z-coordinate of the IMU. Units are in gyro units.
            uint16_t standard_dev { 0 }; ///< standard deviation of IMU noise. Units are in gyro units.
        };

        /**
         * @struct FieldOfViewReport
         * @brief A report of the beacons in the field of view.
         *
         * This structure contains information about which beacons are in the field of view of the sensor, and if they are seen on 1, 2 or 3 optical sensors.
         */
        struct SDS_FALCON_API FieldOfViewReport {
            std::map<uint32_t, int> seenBeacons; ///< Map of beacon ids to the number of optical sensors that the beacon was seen on.
        };

        /**
         * @enum DataPlaybackMode
         * @brief An enum representing mode of the data playback feature.
         *
         * This enumeration defines the operational mode of the data playback feature.
         */
        enum class DataPlaybackMode {
            NONE = 1,   ///< Real-time mode. Playback and recording are disabled.
            RECORD,     ///< Real-time mode with recording enabled.
            PLAYBACK,   ///< Playback mode. Recorded data is replayed instead of live sensor data. Playback begins automatically during initialize(); start() is not required.
        };

        /**
        * @struct DataPlaybackSettings
        * @brief The configuration settings of the data playback feature
        *
        * When mode is NONE then the data playback feature is disabled.
        * When mode is RECORD then data is saved to the path provided.
        * When mode is PLAYBACK then data is read from the path provided and fed to the algorithm. 
        */
        struct SDS_FALCON_API DataPlaybackSettings {
            DataPlaybackMode mode { DataPlaybackMode::NONE }; ///< playback mode.
            std::string path { "" };                          ///< full path to file to log data to, default value of empty string will not log data. (Optional)
        };

        /**
         * @struct Config
         * @brief A structure representing configuration settings for the tracking.
         *
         * This structure contains configuration settings for the tracking.
         * The only required data is the networkConfig, other fields can be left to defaults.
         * To receive 6DOF data, the map member must also be populated.
         */
        struct SDS_FALCON_API Config {
            NetworkConfig networkConfig;             ///< network configuratin for connecting to the sensor. (Required)
            std::vector<Beacon> map;                 ///< vector of beacons for Sixdof tracking. (Required for receiving 6DOF data)
            std::vector<GyroOffset> gyroCalibration; ///< vector of previously recorded gyro calibration data. (Optional)
            bool skip_gyro_calibration { true };     ///< set to true to skip gyro calibration, this will speed up the initialization process.
            bool no_gyro_mode { false };             ///< flag to disable gyro sensor, this mode should only be used in select cases, please contact the Sixdof team for guidance. (Optional)
        };

        /**
         * @brief Save a configuration file.
         * 
         * Save an XML file with the configuration settings.
         * This enables the user to save a file containing the configuration.
         * An exception is thrown if there is an error in saving the file.
         * 
         * @param path The desired path of the file containing the configuration.
         * @param config The configuration.
         */
        SDS_FALCON_API void saveConfig(const std::string& path, const Config& config);

        /**
         * @brief Load a configuration file.
         * 
         * Load an XML file with the configuration settings.
         * This enables the user to load a file containing the configuration instead of entering it manually.
         * It can only be called before initialize is called. 
         * An exception is thrown if there is an error in loading the file.
         * 
         * @param path The path to the file containing the configuration.
         * @return Config
         */
        SDS_FALCON_API Config loadConfig(const std::string& path);

        /**
         * @brief Load a map file.
         * 
         * Load an XML file with the map settings for the configuration of 6DOF tracking.
         * This enables the user to load a file containing the map configuration instead of entering it manually.
         * An exception is thrown if there is an error in loading the file.
         * Note: This function only loads and returns the map data; it does not assign the result to the Config::map member.
         * 
         * @param path The path to the file containing the map configuration.
         * @return The map as a vector of Beacon objects.
         */
        SDS_FALCON_API std::vector<Beacon> loadMap(const std::string& path);

        /**
         * @struct ConfigGyroCalibration
         * @brief A structure representing configuration settings for gyro calibration.
         *
         * This structure contains configuration settings for gyro calibration,
         * all members can be left to defaults.
         */
        struct SDS_FALCON_API ConfigGyroCalibration {
            int duration_seconds { 30 }; ///< duration of gyro calibration capture in seconds.
        };

        /**
         * @typedef CallbackHandle
         * @brief A handle representing a registered callback function.
         *
         * This typedef represents a callback handle used to identify registered callback functions.
         * It is used to remove a specific callback.
         */
        typedef uint32_t CallbackHandle;

        /**
         * @class FalconManager
         * @brief Class for managing the Sixdof Falcon tracking system.
         *
         * This class provides an interface for managing Sixdof Falcon tracking.
         * The correct order of function calls is first initialize, then start.
         * 
         * There are two ways to get tracking data and status messages out of the FalconManager:\n
         *    1) Get functions. these will return the most recent data point. However the getStatusMessage function will return all messages in the order they appeared.\n
         *    2) Function callbacks. The callbacks provided will get called each time a new data point is provided.
         * 
         * It is recommended to use function callbacks for status messages to ensure all messages are received immediately. 
         * It is also recommended to register for status messages before initializing the network, this way we can get all messages from the initialization stage.
         * Each function callback is run on its own thread and it only gets updated once it has finished processing its current data point.
         * This means that long running function callbacks may miss data points that got overshadowed by new data, this is by design.
         * 
         * The FalconManager also allows for the client to do gyro calibration before starting, see the getGyroCalibration() function.
         * 
         */
        class SDS_FALCON_API FalconManager {
        public:
            virtual ~FalconManager() { }
           
            /**
             * @brief Initialize the Sixdof Falcon tracking system.
             *
             * Initializes the tracking system using the provided configuration.
             * Data playback is disabled.
             *
             * @param Config The configuration.
             * @return True if initialization was successful, false otherwise.
             */
            virtual bool initialize(const Config&) = 0;

            /**
             * @brief Initialize the Sixdof Falcon tracking system with data playback settings.
             *
             * Initializes the tracking system using the provided configuration and data playback settings.
             * This overload allows enabling recording or playback of data during initialization.
             *
             * @param Config The configuration.
             * @param DataPlaybackSettings The data playback settings (none, record, or playback).
             * @return True if initialization was successful, false otherwise.
             */
            virtual bool initialize(const Config&, const DataPlaybackSettings&) = 0;

            /**
             * @brief Starts tracking.
             */
            virtual void start() = 0;

            /**
             * @brief Stops tracking, temporarily.
             */
            virtual void stop() = 0;

             /**
             * @brief Gets the firmware version of the sensor.
             */
            virtual Version getFirmwareVersion() = 0;

            /**
             * @brief Perform gyro calibration and retrieve the calibration results.
             *
             * @warning This function must be called only after the sensor has been initialized,
             *          as it requires an active network connection to the device.
             *
             * This function performs a gyro calibration procedure on the sensor.
             * During calibration, the sensor must remain completely stationary.
             *
             * The resulting calibration data is stored internally and is also returned to the caller. 
             * There are two supported usage patterns:
             *
             * 1. **Reuse calibration in a later initialization (recommended for testing):**
             *    Save the returned calibration data and provide it to the @c gyro_calibration member 
             *    of @c Config during a future initialization. This allows calibration to be performed in
             *    advance, eliminating the need for the sensor to be stationary during test startup.
             *
             * 2. **Immediate use without saving the output:**
             *    The returned calibration data may be ignored. Since the calibration is
             *    stored internally, tracking can be started immediately after this call.
             *    In this case, the sensor must be stationary at startup, immediately
             *    following initialization.
             *
             * @param ConfigGyroCalibration Configuration parameters for the gyro calibration procedure.
             * @return A vector of @c GyroOffset objects containing the gyro calibration data.
             */
            virtual std::vector<GyroOffset> getGyroCalibration(const ConfigGyroCalibration&) = 0;

            virtual Pose6Dof getPose6Dof() = 0; ///< get most recent Pose6Dof.

            virtual RelativeAngleCollection getRelativeAngleCollection() = 0; ///< get most recent RelativeAngle.

            virtual StatusMessage getStatusMessage() = 0; ///< get first StatusMessage that was called but not received yet.

            virtual DynamicMessage getDynamicMessage() = 0; ///< get most recent dynamic message.

            virtual FieldOfViewReport getFieldOfViewReport() = 0; ///< get most recent field of view report.

            virtual CurrentShutter getCurrentShutter() = 0; ///< get most recent shutter value.

            virtual CallbackHandle registerPose6DofCallback(const std::function<void(Pose6Dof)>&) = 0; ///< register a function callback that takes Pose6Dof as an input.

            virtual CallbackHandle registerRelativeAngleCallback(const std::function<void(RelativeAngleCollection)>&) = 0; ///< register a function callback that takes RelativeAngleCollection as an input.

            virtual CallbackHandle registerMessageCallback(const std::function<void(StatusMessage)>&) = 0; ///< register a function callback that takes StatusMessage as an input.

            virtual CallbackHandle registerDynamicMessageCallback(const std::function<void(DynamicMessage)>&) = 0; ///< register a function callback that takes DynamicMessage as an input.

            virtual CallbackHandle registerFieldOfViewReportCallback(const std::function<void(FieldOfViewReport)>&) = 0; ///< register a function callback that takes FieldOfViewReport as an input.

            virtual CallbackHandle registerHeartbeatCallback(const std::function<void(Heartbeat)>&) = 0; ///< register a function callback that takes Heartbeat as an input.

            virtual CallbackHandle registerCurrentShutterCallback(const std::function<void(CurrentShutter)>&) = 0; ///< register a function callback that takes a CurrentShutter as an input.

            virtual void removePose6DofCallback(const CallbackHandle&) = 0; ///<< remove a Pose6Dof function callback.

            virtual void removeRelativeAngleCallback(const CallbackHandle&) = 0; ///<< remove a RelativeAngle function callback.

            virtual void removeMessageCallback(const CallbackHandle&) = 0; ///<< remove a StatusMessage function callback.

            virtual void removeDynamicMessageCallback(const CallbackHandle&) = 0; ///< remove a DynamicMessage function callback.

            virtual void removeFieldOfViewCallback(const CallbackHandle&) = 0; ///<< remove a FieldOfViewReport function callback.

            virtual void removeHeartbeatCallback(const CallbackHandle&) = 0; ///<< remove a Heartbeat function callback.
        
            virtual void removeCurrentShutterCallback(const CallbackHandle&) = 0; ///<< remove a Current Shutter function callback. 

            /**
             * @brief Dynamically swap the map.
             *
             * This enables the user to swap the map during execution.
             * It can only be called after initialize has already been called.
             *
             * @param map The new beacon positions.
             */
            virtual void swapMap(const std::vector<Beacon>& map) = 0;

            /**
             * @brief Set custom shutter control logic.
             * 
             * Set custom shutter control logic, from source code. 
             * This function can be called dynamically at runtime.
             */
            virtual void setShutterControl(std::shared_ptr<IShutterControl> shutterControl) = 0;

            /**
             * @brief Set custom shutter control logic.
             * 
             * Set custom shutter control logic, from a compiled plugin. 
             * This function can be called dynamically at runtime. 
             * It should be called after the tracking is initialized.
             * 
             * A valid plugin is in the form of a shared library (.dll on Windows, and .so on Linux), where the following function is implemented:
             * extern "C" IShutterControl* createShutterControl();
             */
            void loadShutterPlugin(const std::string& path);

            /**
             * @brief Set the shutter control logic to the default 
             * 
             * This function can be called dynamically on runtime.
             * If the default shutter logic has been overriden, use this function to return to the default.
             */
            void loadDefaultShutterControl();
        };

        /**
         * @brief Create an instance of FalconManager.
         *
         * This function creates a new instance of the FalconManager.
         * When the FalconManager goes out of scope it will stop the sensor and be cleaned up automatically.
         *
         * @return A unique pointer to a new instance of FalconManager.
         */
        SDS_FALCON_API std::unique_ptr<FalconManager> createFalconManager();
    }
} 
