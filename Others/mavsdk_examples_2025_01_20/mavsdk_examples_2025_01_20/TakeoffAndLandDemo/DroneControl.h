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
    InitHorizontalAlignment,
    HorizontalAlignment,
    InitVerticalDescent,
    VerticalDescent,
    InitGuidedLanding,
    GuidedLandingHorizontal,
    GuidedLandingDescent,
    Land,
    Finished
};

/*
    This class contains the drone control logic for an assisted landing using both RelativeBeacon and Sixdof tracking modes.
    Frist the Relative Beacons estimate is used for the high to low descent, then Sixdof tracking is used to privide a precision landing.
*/
class DroneController {
public:
    DroneController(mavsdk::System& drone, Sds::Falcon::FalconManager& guide);
    ~DroneController() = default;

    // runs the drone control algorithm, including takeoff and landing.
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

    int guided_descent_iteration_counter = 0;    

    /* Flight Parameters */
    const double TAKE_OFF_ALTITUDE = 2.0; // m
    const double TAKE_OFF_ALTITUDE_EPS = 0.2; // m
    const double TRANSITION_ELEVATION = 1.0; // m
    const double TRANSITION_ELEVATION_EPS = 0.05; // m
    const double HORIZONTAL_ALIGNMENT_EPS = 0.05; // m
    const double GUIDED_HORIZONTAL_ALIGNMENT_EPS = 0.05; // m
    const double GUIDED_HEADING_ALIGNMENT_EPS = 7.5; // deg
    const double LAND_ELEVATION = 0.30; // m
    const double LAND_ELEVATION_EPS = 0.02; // m
    const double MAX_SPEED = 1.2; // m / s
    const double CLIMB_TIME = 6.0; // s
    const float GUIDED_DESCENT_ITTERATIONS = 1000;

    struct PositionNED {
        double N;
        double E;
        double D;
    };
    
    /* Utility Functions */
    // convert the current RelativeBeacon pose to NED
    std::optional<DroneController::PositionNED> relLight_wrt_map_to_ned(Sds::Falcon::PoseRelativeBeacon relativeBeacon);
    
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
