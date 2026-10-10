#include "DroneControl.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>

DroneController::DroneController(mavsdk::System& drone, Sds::Falcon::FalconManager& guide)
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
        


void DroneController::run() {
    while (true) {
        switch (state) {

            case ExecutionState::Init: {
                std::cout << "Setting max speed to " << MAX_SPEED << "m/s" << std::endl;
                action->set_maximum_speed(MAX_SPEED);

                // Arm Process
                std::cout << "-- Arming" << std::endl;
                // Wait until health is ok and vehicle is ready to arm
                while(telemetry->health_all_ok() != true){
                    std::cout << "Vehicle not ready to arm..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }

                // Arm Drone
                const mavsdk::Action::Result arm_result = action->arm();
                if (arm_result != mavsdk::Action::Result::Success) {
                    std::cout << "Arming failed:" << arm_result << std::endl;
                    return;
                }
                state = ExecutionState::Takeoff;
                break;
            }

            case ExecutionState::Takeoff: {
                std::cout << "Take off to " << TAKE_OFF_ALTITUDE << "m" << std::endl;
                init_time = std::chrono::steady_clock::now();
                // Set takeoff altitude
                action->set_takeoff_altitude(TAKE_OFF_ALTITUDE);
                // Takeoff
                const mavsdk::Action::Result takeoff_result = action->takeoff();
                if (takeoff_result != mavsdk::Action::Result::Success) {
                    std::cout << "Takeoff failed:" << takeoff_result << std::endl;
                    return; 
                }
                state = ExecutionState::Climb;
                break;
            }

            case ExecutionState::Climb: {
                // Climb until time elapsed equals CLIMB_TIME 
                auto curr_time = std::chrono::steady_clock::now();
                if((curr_time - init_time).count() * 1.0e-9 >= CLIMB_TIME){
                    std::cout << "Reached Climb Time" << std::endl;
                }
                state = ExecutionState::InitHorizontalAlignment;
                break;
            }

            case ExecutionState::InitHorizontalAlignment: {
                // Set tracking mode to Relative light
                guide.setTrackingMode(Sds::Falcon::TrackingMode::RelativeBeacon); 

                // Start offboard mode  
                offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
                offboard->start();

                state = ExecutionState::HorizontalAlignment;
                break;
            }

            case ExecutionState::HorizontalAlignment: {
                // Align the drone with the landing pad by moving it in the horizontal plane
    
                // Get relative light position 
                Sds::Falcon::PoseRelativeBeaconCollection relativeBeaconCol = guide.getPoseRelativeBeaconCollection();
                // In this example we assume you can only see the large beacon
                // another way to do this is to identify the beacon you are looking for by its id
                if (relativeBeaconCol.size() != 1) {
                    std::cout << "Invalid Relative Beacon" << std::endl;
                    continue;
                }

                PoseRelativeBeacon relativeBeacon = relativeBeaconCol[0];

                // Spoof relative light position to go to the takeoff altitude 
                relativeBeacon.y -= TAKE_OFF_ALTITUDE;

                // Convert relative light position to PositionNED coordinates
                std::optional<DroneController::PositionNED> positionNed = relLight_wrt_map_to_ned(relativeBeacon); 
                
                // If the drone is within a certain distance of being directly above the map, enter vertical descent 
                if (std::sqrt(std::pow(relativeBeacon.x, 2) + std::pow(relativeBeacon.z, 2)) < HORIZONTAL_ALIGNMENT_EPS) {
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
                    state = ExecutionState::InitVerticalDescent;
                } else {
                    // If the drone still has not aligned above the map, tell it to go to that position 
                    offboard->set_position_ned({(float)positionNed.value().N, (float)positionNed.value().E, (float)positionNed.value().D, 0.0});
                }
                break;
            }

            case ExecutionState::InitVerticalDescent: {
                std::cout << "Starting Vertical Descent" << std::endl;
                state = ExecutionState::VerticalDescent;
                break;
            }

            case ExecutionState::VerticalDescent: {
                Sds::Falcon::PoseRelativeBeaconCollection relativeBeaconCol = guide.getPoseRelativeBeaconCollection();
                // In this example we assume you can only see the large beacon
                // another way to do this is to identify the beacon you are looking for by its id
                if (relativeBeaconCol.size() != 1) {
                    std::cout << "Invalid Relative Beacon" << std::endl;
                    continue;
                }

                PoseRelativeBeacon relativeBeacon = relativeBeaconCol[0];

                // If the drone has descended to the transition elevation, switch to guided landing with sixdof tracking 
                if (std::abs(relativeBeacon.y) <= TRANSITION_ELEVATION + TRANSITION_ELEVATION_EPS) {
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
                    state = ExecutionState::InitGuidedLanding;
                } else {
                    // Spoof relative light position to send it to the transition elevation 
                    relativeBeacon.y -= TRANSITION_ELEVATION;

                    // Convert relative light position to PositionNED coordinates
                    std::optional<DroneController::PositionNED> positionNed = relLight_wrt_map_to_ned(relativeBeacon); 
                    
                    // Send the drone to that position 
                    offboard->set_position_ned({(float)positionNed.value().N, (float)positionNed.value().E, (float)positionNed.value().D, 0.0});
                }
                break;
            }

            case ExecutionState::InitGuidedLanding: {
                // Change the tracking mode to sixdof tracking 
                guide.setTrackingMode(Sds::Falcon::TrackingMode::Sixdof); 
                offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
                state = ExecutionState::GuidedLandingHorizontal;
            }

            case ExecutionState::GuidedLandingHorizontal: {
                // Get the sixdof position of the drone from the sensor and check that it is valid
                Sds::Falcon::Pose6Dof sixdof = guide.getPose6Dof(); 
                if(!sixdof.valid){
                    std::cout << "No Sixdof estimate" << std::endl;
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});                    
                    continue;
                }

                // Spoof sixdof so that it stays at TRANSITION_ELEVATION
                sixdof.y += TRANSITION_ELEVATION;

                // Convert sixdof to PositionNED coordinates 
                std::optional<Offboard::PositionNedYaw> position_ned = sixdof_wrt_map_to_nedy(sixdof);

                // Use sixdof quaternion to find heading pitch roll 
                Eigen::Quaterniond quat(sixdof.qw, sixdof.qx, sixdof.qy, sixdof.qz);
                Eigen::Vector3d hpr = quaternion_to_hpr(quat);

                // If the drone is within  a certain threshold of being aligned directly over the map, Enter guided landing descent 
                if (std::sqrt(std::pow(sixdof.x, 2) + std::pow(sixdof.z, 2)) < GUIDED_HORIZONTAL_ALIGNMENT_EPS  && std::abs(hpr[2]) <= deg_to_rad(GUIDED_HEADING_ALIGNMENT_EPS)) {
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});         
                    state = ExecutionState::GuidedLandingDescent;
                } else {
                    offboard->set_position_ned(position_ned.value());
                }
            }

            case ExecutionState::GuidedLandingDescent: {

                Sds::Falcon::Pose6Dof sixdof = guide.getPose6Dof(); 
                if(!sixdof.valid){
                    std::cout << "No Sixdof estimate" << std::endl;
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});                    
                    continue;
                }
                
                // Iterate the counter to slowly decrease elevation and facilitate a smooth landing 
                guided_descent_iteration_counter += 1;

                // save hieght before spoofing
                double height = sixdof.y;

                // Spoof sixdof to go to the elevation currently set by the loop iteration, this will decrease every loop and slowly land he drone 
                double spoof = TRANSITION_ELEVATION * (1.0 - std::min((double)guided_descent_iteration_counter / GUIDED_DESCENT_ITTERATIONS, 1.0));
                sixdof.y += spoof;

                // Convert sixdof to PositionNED coordinates 
                std::optional<Offboard::PositionNedYaw> position_ned = sixdof_wrt_map_to_nedy(sixdof);
                if (position_ned == std::nullopt) {
                    std::cout << "No Nedy Value" << std::endl;
                    continue;
                }

                // If the drone has decsended to the land elevation, Enter the land state 
                if (std::abs(height) <= LAND_ELEVATION) {
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});         
                    state = ExecutionState::Land;
                } else {    
                    offboard->set_position_ned(position_ned.value());
                }
                break;
            }

            case ExecutionState::Land: {
                std::cout << "Landing..." << std::endl;
                // Exit offboard mode
                offboard->stop();
                
                // Land the drone
                action->land();
                std::this_thread::sleep_for(std::chrono::seconds(5));
                
                // Kill the drone 
                action->kill();

                // Stop the sensor
                guide.setTrackingMode(TrackingMode::TrackingOFF);

                state = ExecutionState::Finished;
                break;
            }
            case ExecutionState::Finished: {
                std::cout << "Mission complete" << std::endl;
                return; 
            }
        }
    }
}

std::optional<DroneController::PositionNED> DroneController::relLight_wrt_map_to_ned(Sds::Falcon::PoseRelativeBeacon relativeBeacon) {
    Eigen::Vector4d homogenous;
    homogenous << relativeBeacon.x, relativeBeacon.y, relativeBeacon.z, 1.0;
    
    // this matrix is determined based on how the sensor is mounted on the drone
    Eigen::Matrix4d sensor_wrt_drone;
    sensor_wrt_drone << 0, 0, 1, 0,
                        1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 0, 1;
    
    Eigen::Matrix4d drone_wrt_ned = Eigen::Matrix4d::Identity();

    {   // anonomus scope used to release mutex when done
        std::lock_guard<std::mutex> lock(currPositionMutex);
        if (currPosition) {
            drone_wrt_ned(0, 3) = currPosition->north_m;
            drone_wrt_ned(1, 3) = currPosition->east_m;
            drone_wrt_ned(2, 3) = currPosition->down_m;
        } else {
            // Case where failure to get curr pos
            return std::nullopt;; // std optional
        }
    }

    {   // anonomus scope used to release mutex when done
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
    DroneController::PositionNED positionNed; 
    positionNed.N = homogenousNed(0);
    positionNed.E = homogenousNed(1);
    positionNed.D = homogenousNed(2);
    return positionNed;
}

std::optional<Offboard::PositionNedYaw> DroneController::sixdof_wrt_map_to_nedy(Sds::Falcon::Pose6Dof sixdof){
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    double sixdof_wrt_target[] = {sixdof.x, sixdof.y, sixdof.z, sixdof.qx};

    Eigen::Transform<double, 3, Eigen::Isometry> sensor_wrt_target;
    sensor_wrt_target.translation()(0) = sixdof.x;
    sensor_wrt_target.translation()(1) = sixdof.y;
    sensor_wrt_target.translation()(2) = sixdof.z;
    
    Eigen::Quaterniond quat(sixdof.qw, sixdof.qx, sixdof.qy, sixdof.qz);
    quat.normalize();
    sensor_wrt_target.linear() = quat.toRotationMatrix();

    Eigen::Transform<double, 3, Eigen::Isometry> target_wrt_sensor = sensor_wrt_target.inverse();

    // this matrix is determined based on how the sensor is mounted on the drone
    Eigen::Matrix4d sensor_wrt_drone;
    sensor_wrt_drone << 0, 0, 1, 0,
                        1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 0, 1;

    // Get NEDY Coordinates
    Eigen::Matrix4d drone_wrt_ned = Eigen::Matrix4d::Identity();

    {   // anonomus scope used to release mutex when done
        std::lock_guard<std::mutex> lock(currPositionMutex);
        if (currPosition) {
            drone_wrt_ned(0, 3) = currPosition->north_m;
            drone_wrt_ned(1, 3) = currPosition->east_m;
            drone_wrt_ned(2, 3) = currPosition->down_m;
        } else {
            // Case where failure to get curr pos
            return std::nullopt;; // std optional
        }
    }

    {   // anonomus scope used to release mutex when done
        std::lock_guard<std::mutex> lock(currHeadingMutex);
        if (currHeading) {
            Eigen::AngleAxisd rot(deg_to_rad(*currHeading), Eigen::Vector3d::UnitZ()); // todo conver to rad
            drone_wrt_ned.block(0,0,3,3) = rot.toRotationMatrix();
        } else {
            return std::nullopt;
        }
    }

    auto target_wrt_ned = drone_wrt_ned * sensor_wrt_drone * target_wrt_sensor;

    Offboard::PositionNedYaw nedy;
    nedy.north_m = target_wrt_ned.translation()(0);
    nedy.east_m = target_wrt_ned.translation()(1);
    nedy.down_m = target_wrt_ned.translation()(2);

    Eigen::Quaterniond q_ned(target_wrt_ned.rotation());
    Eigen::Vector3d hpr = quaternion_to_hpr(q_ned);
    nedy.yaw_deg = rad_to_deg(hpr(0)); 

    return nedy;
}


int DroneController::sign(double val) {
    if(val > 0){
        return 1;
    } else if(val < 0){
        return -1;
    } else{
        return 0;
    }
}

double DroneController::rad_to_deg(double radians){
    Eigen::Rotation2D<double> rotation(radians);
    double degrees = rotation.angle() * 180.0 / M_PI;
    if (degrees < 0){
        degrees += 360.0;
    }
    return degrees;
}

double DroneController::deg_to_rad(double degrees){
    return degrees * M_PI / 180;
}


Eigen::Vector3d DroneController::quaternion_to_hpr(Eigen::Quaterniond q) {
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
