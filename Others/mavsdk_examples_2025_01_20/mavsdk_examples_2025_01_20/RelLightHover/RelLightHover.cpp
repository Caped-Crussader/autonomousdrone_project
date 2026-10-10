#include "RelLightHover.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>

RelLightHover::RelLightHover(mavsdk::System& drone, Sds::Falcon::FalconManager& guide) 
    : drone(drone), guide(guide), state(ExecutionState::Init), init_time(std::chrono::steady_clock::now())
{
    telemetry = std::make_shared<mavsdk::Telemetry>(drone);
    offboard = std::make_shared<mavsdk::Offboard>(drone);
    action = std::make_shared<mavsdk::Action>(drone);
    
    telemetry->subscribe_position_velocity_ned([&](const mavsdk::Telemetry::PositionVelocityNed& posVelNed) {
        std::lock_guard<std::mutex> lock(currPositionMutex);
        currPosition = posVelNed.position;   
    });

    telemetry->subscribe_heading([&](const mavsdk::Telemetry::Heading& heading) {
        std::lock_guard<std::mutex> lock(currHeadingMutex);
        currHeading = heading.heading_deg;
    });
}

void RelLightHover::run() {
    while (true) {
        switch (state) {
            case ExecutionState::Init: {
                // Set max speed
                std::cout << "Setting max speed to " << MAX_SPEED << "m/s" << std::endl;
                action->set_maximum_speed(MAX_SPEED);

                mavsdk::Telemetry::Health health = telemetry->health();
                // Arm Process
                std::cout << "-- Arming" << std::endl;
                // Wait until health is ok and vehicle is ready to arm
                while(telemetry->health_all_ok() != true){
                    std::cout << "Vehicle not ready to arm..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }

                // Arm Drone
                action->arm();
                state = ExecutionState::Takeoff;
                break;
            }

            case ExecutionState::Takeoff: {
                std::cout << "Takeoff to " << HOVER_ALTITUDE << "m" << std::endl;

                // Set takeoff altitude
                action->set_takeoff_altitude(HOVER_ALTITUDE);

                // Takeoff and check for success of takeoff
                const mavsdk::Action::Result takeoff_result = action->takeoff();
                if (takeoff_result != mavsdk::Action::Result::Success) {
                    std::cout << "Takeoff failed:" << std::endl;
                    return;  
                }

                state = ExecutionState::Climb;
                break;
            }

            case ExecutionState::Climb: {
                auto curr_time = std::chrono::steady_clock::now();

                // Time Management
		        if((curr_time - init_time).count() * 1.0e-9 >= CLIMB_TIME){
		            std::cout << "Reached Climb Time" << std::endl;
                    state = ExecutionState::InitGoTo;
                }

                state = ExecutionState::InitGoTo;
                break;
            }

            case ExecutionState::InitGoTo: {
                // Reset hover time
                init_time = std::chrono::steady_clock::now();

                // Set Tracking Mode
                guide.setTrackingMode(Sds::Falcon::TrackingMode::RelativeBeacon);

                // Start offboard mode
                offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
                offboard->start();

                state = ExecutionState::GoTo;
                break;
            }

            case ExecutionState::GoTo: {
                // continually send commands to move to hover location until HOVER_TIME elapses

                PoseRelativeBeaconCollection relativeBeaconCol = guide.getPoseRelativeBeaconCollection();
                
                // in this example there is exactly one beacon
                if (relativeBeaconCol.size() != 1) {
                    std::cout << "Invalid Relative Beacon" << std::endl;
                    continue;
                }

                PoseRelativeBeacon relativeBeacon = relativeBeaconCol[0];

                relativeBeacon.y += HOVER_ALTITUDE;
                std::optional<RelLightHover::PositionNED> positionNed = relLight_wrt_map_to_ned(relativeBeacon); 
                
                double heading = 0;
                std::lock_guard<std::mutex> lock(currHeadingMutex);
                if (currHeading) {
                    heading = *currHeading; 
                }

                offboard->set_position_ned({(float)positionNed.value().N, (float)positionNed.value().E, (float)positionNed.value().D, (float)heading});

                // Wait for preset hover time
                auto curr_time = std::chrono::steady_clock::now();
                if((init_time - curr_time).count() * 1.0e-9 >= HOVER_TIME){
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
                    state = ExecutionState::Land;
                }

                break;
            }

            case ExecutionState::Land: {
                // End Offboard
                offboard->stop();

                // Land Drone
                action->land();
                std::this_thread::sleep_for(std::chrono::seconds(5));

                // Kill Drone
                action->kill();
                
                // Stop sensor
                guide.setTrackingMode(TrackingMode::TrackingOFF);

                state = ExecutionState::Finished;
                break;
            }

            case ExecutionState::Finished: {
                std::cout << "Mission Complete" << std::endl;
                return;
            }
        }
    }
}

std::optional<RelLightHover::PositionNED> RelLightHover::relLight_wrt_map_to_ned(Sds::Falcon::PoseRelativeBeacon relativeBeacon) {
    
    Eigen::Vector4d homogenous;
    homogenous << relativeBeacon.x, relativeBeacon.y, relativeBeacon.z, 1.0;
    
    // this matrix is determined based on how the sensor is mounted on the drone
    Eigen::Matrix4d sensor_wrt_drone;
    sensor_wrt_drone << 0, 0, 1, 0,
                        1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 0, 1;

    
    Eigen::Matrix4d drone_wrt_ned = Eigen::Matrix4d::Identity();

    {
        std::lock_guard<std::mutex> lock(currPositionMutex);
        if (currPosition) {
            drone_wrt_ned(0, 4) = currPosition->north_m;
            drone_wrt_ned(1, 4) = currPosition->east_m;
            drone_wrt_ned(2, 4) = currPosition->down_m;
        } else {
            // Case where failure to get curr pos
            return std::nullopt;
        }
    }

    {
        std::lock_guard<std::mutex> lock(currHeadingMutex);
        if (currHeading) {
            Eigen::AngleAxisd rot(deg_to_rad(*currHeading), Eigen::Vector3d::UnitZ()); 
            drone_wrt_ned.block(0,0,3,3) = rot.toRotationMatrix();
        } else {
            return std::nullopt;
        }
    }

    Eigen::Vector4d homogenousNed = drone_wrt_ned * sensor_wrt_drone * homogenous;
    homogenousNed /= homogenousNed(3);
    RelLightHover::PositionNED positionNed; 
    positionNed.N = homogenousNed(0);
    positionNed.E = homogenousNed(1);
    positionNed.D = homogenousNed(2);
    return positionNed;
}

int RelLightHover::sign(double val) {
    if(val > 0){
        return 1;
    } else if(val < 0){
        return -1;
    } else{
        return 0;
    }
}

double RelLightHover::rad_to_deg(double radians){
    Eigen::Rotation2D<double> rotation(radians);
    double degrees = rotation.angle() * 180.0 / M_PI;
    if (degrees < 0){
        degrees += 360.0;
    }
    return degrees;
}

double RelLightHover::deg_to_rad(double degrees){
    return degrees * M_PI / 180;
}


Eigen::Vector3d RelLightHover::quaternion_to_hpr(Eigen::Quaterniond q) {
    // Normalizes quaternion on input
    q.normalize();

    double sinr_cosp = 2.0 * (q.w() * q.y() - q.x() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    double r = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double p;
    if (abs(sinp) >= 1.0) {
        p = M_PI / 2.0 * sign(sinp);
    } else {
        p = asin(sinp);
    }

    double sinh_cosp = 2.0 * (q.w() * q.z() - q.x() * q.y());
    double cosh_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.z() * q.z());
    double h = atan2(sinh_cosp, cosh_cosp);

    Eigen::Vector3d hpr;
    hpr << h, p, r;
    return hpr;
}
