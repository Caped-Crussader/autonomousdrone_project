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
    InitGuidedLanding,
    GuidedLandingHorizontal,
    GuidedLandingDescent,
    Land,
    Finished
};

/*
    This class contains the drone control logic for a assisted hover and landing using Sixdof tracking mode.
*/
class DroneSixdofHover {
public:
    DroneSixdofHover(std::shared_ptr<mavsdk::System> drone, Sds::Falcon::FalconManager& guide);
    ~DroneSixdofHover() = default; 
    
    // runs the drone control algorithm, including takeoff, hover and land.
    void run();

private: 
    /* Inputs */
    std::shared_ptr<mavsdk::System> drone;
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

    int guided_descent_counter = 0;

    /* Flight Parameters */
    const float MAX_SPEED = 1.75; // m/s
    const float HOVER_ALTITUDE = 1.5; // m
    const int CLIMB_TIME = 8; // sec
    const int HOVER_TIME = 15; // sec
    const double LAND_ELEVATION = 0.30; // m
    const double GUIDED_HORIZONTAL_ALIGNMENT_EPS = 0.10; // m
    const double GUIDED_HEADING_ALIGNMENT_EPS = 7.5;   // deg
    const double TRANSITION_ELEVATION = 1.5;           // m
    const double TRANSITION_ELEVATION_EPS = 0.05;      // m
    const int GUIDED_DESCENT_TOTAL_ITERATIONS = 1000;

    /* Utility Functions */
    // convert the current Sixdof pose of the sensor to NEDY coordinates of the drone
    std::optional<Offboard::PositionNedYaw> sixdof_wrt_map_to_nedy(Sds::Falcon::Pose6Dof sixdof);

    // convert quaternion representation of 3D angles to Euler angle representation (heading, pitch, roll)
    static Eigen::Vector3d quaternion_to_hpr(Eigen::Quaterniond q);

    // get the sign of a value
    static int sign(double val);

    // convert radians to degrees
    double rad_to_deg(double radians);

    // convert degrees to radians
    double deg_to_rad(double degrees);
};
