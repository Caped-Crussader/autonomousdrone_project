#include "DroneSixdofHover.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>

DroneSixdofHover::DroneSixdofHover(std::shared_ptr<mavsdk::System> drone, Sds::Falcon::FalconManager& guide) 
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

void DroneSixdofHover::run() {
    while (true) {
        switch (state) {

            case ExecutionState::Init: {
                std::cout << "Setting max speed to " << MAX_SPEED << "m/s" << std::endl;
                action->set_maximum_speed(MAX_SPEED);
                
                // Arm Process
                mavsdk::Telemetry::Health health = telemetry->health();
                std::cout << "-- Arming" << std::endl;
                
                // Wait until health is ok and vehicle is ready to arm
                while(telemetry->health_all_ok() != true){
                    std::cout << "Vehicle not ready to arm..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                action->arm();

                state = ExecutionState::Takeoff;
                break;
            }

            case ExecutionState::Takeoff: {
                std::cout << "Takeoff to " << HOVER_ALTITUDE << "m" << std::endl;
                init_time = std::chrono::steady_clock::now();
                
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
                // Climb from takeoff to hover height, duration of CLIMB_TIME
        		
                auto curr_time = std::chrono::steady_clock::now();

                // Time Management
		        if((curr_time - init_time).count() * 1.0e-9 >= CLIMB_TIME){
		            std::cout << "Reached Climb Time" << std::endl;
                    state = ExecutionState::InitGoTo;
                }
                break;
            }

            case ExecutionState::InitGoTo: {
                // Reset hover time
                init_time = std::chrono::steady_clock::now();

                // Set Tracking Mode
                guide.setTrackingMode(Sds::Falcon::TrackingMode::Sixdof);

                // Start offboard mode
        		offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
                offboard->start();
		        init_time = std::chrono::steady_clock::now();
                
                state = ExecutionState::GoTo;
                break; 
            }

            case ExecutionState::GoTo: { 
                // continually send commands to move to hover location until HOVER_TIME elapses
                
                // Set Sixdof position, accounting for hover altitude in y variable
                Sds::Falcon::Pose6Dof sixdof = guide.getPose6Dof(); 
                if(!(sixdof.valid)){
                    std::cout << "No Sixdof estimate" << std::endl;
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});           
                    continue;
                }
	
                // Spoof y value to account for desired hover altitude
                sixdof.y += HOVER_ALTITUDE;

                // Set N, E, D, Y coordinates to sixdof_wrt_map_to_nedy
                std::optional<Offboard::PositionNedYaw> position_ned = sixdof_wrt_map_to_nedy(sixdof);
		
                if(!position_ned.has_value()){
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
               		std::cout << "No Ned Value" << std::endl;
		            continue;
                }

                // set drone to align with North
		        position_ned.value().yaw_deg = 0;
		        offboard->set_position_ned(position_ned.value());
		
                // Wait for hover time
                auto curr_time = std::chrono::steady_clock::now();
                if((curr_time - init_time).count() * 1.0e-9 >= HOVER_TIME){
		            std::cout << "Hover Time Completed" << std::endl;
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
                    state = ExecutionState::InitGuidedLanding;
                }

                break; 
            }

            case ExecutionState::InitGuidedLanding: {
		        std::cout << "InitGuidedLanding" << std::endl;
                offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
		                       
                state = ExecutionState::GuidedLandingHorizontal;
                break;
            }

            case ExecutionState::GuidedLandingHorizontal: { 
                // Moves horizontally until directly above the desired landing location

                Sds::Falcon::Pose6Dof sixdof = guide.getPose6Dof();
                if(!(sixdof.valid)){
                    std::cout << "No Sixdof estimate" << std::endl;
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});                    
                    continue;
                }

                // Spoof sixdof so that it stays at TRANSITION_ELEVATION
                sixdof.y += TRANSITION_ELEVATION;

                std::optional<Offboard::PositionNedYaw> position_ned = sixdof_wrt_map_to_nedy(sixdof);
                if (position_ned == std::nullopt) {
                    std::cout << "No Ned Value" << std::endl;
                    continue;
                }

                double distance_to_alignment = std::sqrt(std::pow(sixdof.x, 2) + std::pow(sixdof.z, 2));
                if (distance_to_alignment < GUIDED_HORIZONTAL_ALIGNMENT_EPS) {
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});         
                    action->set_maximum_speed(0.3);  // Update max speed to slow down descent
                    state = ExecutionState::GuidedLandingDescent;
                } else {
                    position_ned.value().yaw_deg = 0; // align drone to North
                    offboard->set_position_ned(position_ned.value());
                }
                
                break;
            }

            case ExecutionState::GuidedLandingDescent: { 
                // Sends drone down 1/1000 of the desired height per iteration,
                // stopping once spoofed height is less than Land_Elevation

                Sds::Falcon::Pose6Dof sixdof = guide.getPose6Dof(); 
                if(!(sixdof.valid)){
                    std::cout << "No Sixdof estimate" << std::endl;
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});                    
                    continue;
                }

                guided_descent_counter += 1;
                
                // save hight before spoofing
                double height = sixdof.y;

                // Spoof height to desired elevation to move to 
                sixdof.y += TRANSITION_ELEVATION * (1.0 - std::min((double)guided_descent_counter / GUIDED_DESCENT_TOTAL_ITERATIONS, 1.0));

                std::optional<Offboard::PositionNedYaw> position_ned = sixdof_wrt_map_to_nedy(sixdof);
                if (position_ned == std::nullopt) {
                    std::cout << "No Ned Value" << std::endl;
                    continue;
                }

                if (std::abs(height) <= LAND_ELEVATION) {
                    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});         
                    state = ExecutionState::Land;
                } else {    
                    position_ned.value().yaw_deg = 0.0; // align drone to North
                    offboard->set_position_ned(position_ned.value());
                }
            
                break;
            }

            case ExecutionState::Land: { 
                // Ends offboard functionality, lands and kills drone
		        std::cout << "Land" << std::endl;
		        offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
                
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

std::optional<Offboard::PositionNedYaw> DroneSixdofHover::sixdof_wrt_map_to_nedy(Sds::Falcon::Pose6Dof sixdof){
    
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();

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
            return std::nullopt;
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

int DroneSixdofHover::sign(double val) {
    if(val > 0){
        return 1;
    } else if(val < 0){
        return -1;
    } else {
        return 0;
    }
}

double DroneSixdofHover::rad_to_deg(double radians){
    Eigen::Rotation2D<double> rotation(radians);
    double degrees = rotation.angle() * 180.0 / M_PI;
    if (degrees < 0){
        degrees += 360.0;
    }
    return degrees;
}

double DroneSixdofHover::deg_to_rad(double degrees){
    return degrees * M_PI / 180;
}

Eigen::Vector3d DroneSixdofHover::quaternion_to_hpr(Eigen::Quaterniond q) {
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
