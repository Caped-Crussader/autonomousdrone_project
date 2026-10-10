#pragma once

#include "mavsdk/mavsdk.h"
#include "libs/falcon/include/SdsFalcon.h"
#include "libs/MAVSDK-1.4/src/mavsdk/plugins/action/include/plugins/action/action.h"
#include "libs/MAVSDK-1.4/src/mavsdk/plugins/offboard/include/plugins/offboard/offboard.h"
#include "libs/MAVSDK-1.4/src/mavsdk/plugins/telemetry/include/plugins/telemetry/telemetry.h"
#include "libs/MAVSDK-1.4/src/mavsdk/plugins/mission/include/plugins/mission/mission.h"

#include <mutex>
#include <optional>
#include <Eigen/Geometry>

using namespace mavsdk;
using namespace Sds::Falcon;


enum class ExecutionState {
    Init,
    Takeoff,
    Climb,
    InitGoTo,
    GoTo,
    Land,
    Finished
};

/*
    This class contains the drone control logic for a assisted hover and landing using RelativeBeacon tracking mode.
    Note that in RelativeBeacon tracking mode, the heading informaiton of the drone relative to the landing target is unknown.
*/
class RelLightHover {
public:
    RelLightHover(mavsdk::System& drone, Sds::Falcon::FalconManager& guide);
    ~RelLightHover() = default;

    // runs the drone control algorithm, including takeoff, hover and land.
    void run();

private:
    /* Inputs */
    mavsdk::System& drone;
    Sds::Falcon::FalconManager& guide;

    std::shared_ptr<mavsdk::Telemetry> telemetry;
    std::shared_ptr<mavsdk::Offboard> offboard;
    std::shared_ptr<mavsdk::Action> action;

    /* Flight Control State */
    ExecutionState state;
    std::chrono::steady_clock::time_point init_time;

    std::mutex currPositionMutex;
    std::optional<mavsdk::Telemetry::PositionNed> currPosition;

    std::mutex currHeadingMutex;
    std::optional<double> currHeading;

    /* Flight Parameters */
    const float MAX_SPEED = 1.0; // m/s
    const float HOVER_ALTITUDE = 1.5;  // m
    const int CLIMB_TIME = 10;  // sec
    const int HOVER_TIME = 30;  // sec

    struct PositionNED {
        double N;
        double E;
        double D;
    };
    
    /* Utility Functions */
    // convert the current RelativeBeacon pose to NED
    std::optional<RelLightHover::PositionNED> relLight_wrt_map_to_ned(Sds::Falcon::PoseRelativeBeacon relativeBeacon);
    
    // convert quaternion representation of 3D angles to Euler angle representation (heading, pitch, roll)
    static Eigen::Vector3d quaternion_to_hpr(Eigen::Quaterniond q);

    // get the sign of a value
    static int sign(double val);

    // convert radians to degrees
    double rad_to_deg(double radians);

    // convert degrees to radians
    double deg_to_rad(double degrees);
};
