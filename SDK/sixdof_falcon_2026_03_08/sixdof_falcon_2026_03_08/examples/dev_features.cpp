#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsFalcon.h"
#include "DevFeatures.h"

using namespace Sds::Falcon;


// ***************************************************************************
// *                                                                         *
// *                    Changing Dev Features Example                        *
// *                                                                         *
// *   This example demonstrates how to use and change the interal           *
// *   algorithm's development features. Please reference the Sixdof         *
// *   Falcon User Guide for details.                                        *
// *                                                                         *
// ***************************************************************************


int main(int ac, const char *av[]) {

    // Print the SdsFalcon SDK version
    Version version = getVersion();
    std::cout << "Using SdsFalcon version: " << version.getString() << std::endl;

    // Initialize the Falcon Manager
    std::unique_ptr<FalconManager> falconManager = createFalconManager();
    std::cout << "Falcon Manager created" << std::endl;

    // Register callbacks before calling the init function so we can get error messages printed out
    falconManager->registerMessageCallback([](const StatusMessage& msg){
        std::cout << msg.message << std::endl;
    });

    falconManager->registerPose6DofCallback([](const Pose6Dof& sd){
        std::cout << std::fixed << std::setprecision(3)
                  << "X: " << sd.x 
                  << " Y: " << sd.y 
                  << " Z: " << sd.z 
                  << " Qw: " << sd.qw 
                  << " Qx: " << sd.qx 
                  << " Qy: " << sd.qy 
                  << " Qz: " << sd.qz << std::endl;
    });

    // Enter the configuration data for your specific setup
    Config config;

    // Input the sensor id, and the network name
    NetworkConfig networkConfiguration { 0, "Ethernet" };
    config.networkConfig = networkConfiguration;

    // Input the map
    Beacon b1 {0.0, 0.0, 0.0, 1};
    Beacon b2 {0.0, 0.0, 0.0, 2};
    Beacon b3 {0.0, 0.0, 0.0, 3};
    Beacon b4 {0.0, 0.0, 0.0, 4};
    Beacon b5 {0.0, 0.0, 0.0, 5};
    config.map.push_back(b1);
    config.map.push_back(b2);
    config.map.push_back(b3);
    config.map.push_back(b4);
    config.map.push_back(b5);

    // Initialize
    bool success = falconManager->initialize(config);
    if (!success) {
        std::cout << "Falcon Manager failed initialization" << std::endl;
        return 1;
    }

    DevFeatures* devFeatures = createDevFeatures();

    double std_xyz_value = devFeatures->getParam(ParamType::std_xyz);
    std::cout << "std_xyz = " << std_xyz_value << std::endl;
    
    // Start Tracking
    std::cout << "Start Tracking" << std::endl;
    falconManager->start();
    
    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "Setting feature std_xyz = 0.0" << std::endl;
    std::cout << "Move the sensor around. The 6DOF data will show that the sensor barely moves in the XYZ directions, even though you are physically moving it." << std::endl;    
    devFeatures->setParam(ParamType::std_xyz, 0.0);
    std_xyz_value = devFeatures->getParam(ParamType::std_xyz);
    std::cout << "std_xyz = " << std_xyz_value << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "Re-loading the default feature values" << std::endl;
    loadDefaultDevFeatures(devFeatures);
    std_xyz_value = devFeatures->getParam(ParamType::std_xyz);
    std::cout << "std_xyz = " << std_xyz_value << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    falconManager->stop();
    std::cout << "Done" << std::endl;

    return 0;
}