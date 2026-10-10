#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsFalcon.h"

using namespace Sds::Falcon;


// ***************************************************************************
// *                                                                         *
// *                      Field of View Report Example                       *
// *                                                                         *
// *   This example demonstrates how to use the Field of View Report.        *
// *   The FOV Report is a callback that shows which beacons are seen by the * 
// *   sensor. Sending the FOV Report is a debug feature to help users       *
// *   understand what beacons are in the field of view for a given pose.    *
// *   The FOV Report should not be used in production.                      *
// *   It is important to note the order of function calls:                  *
// *   - Create a Falcon Manager                                             *
// *   - Register Callbacks, this is where the FOV Report callback is set    *
// *   - Create the Configuration                                            *
// *   - Initialize (establishes a connection to the sensor)                 *
// *   - Start the Tracking Mode                                             *
// *   - View the outputs from the registered callbacks                      *
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

    // Register the Field of View Report callback
    falconManager->registerFieldOfViewReportCallback([](const FieldOfViewReport& report){
        std::cout << "New Report:" << std::endl;
        for (const auto& kv : report.seenBeacons) {
            std::cout << "Beacon: " << kv.first << " " << kv.second << std::endl;
        }
    });

    // Enter the configuration data for your specific setup
    Config config;

    // Input the sensor id, and the network name
    NetworkConfig networkConfiguration { 0, "Ethernet" };
    config.networkConfig = networkConfiguration;

    // Initialize
    bool success = falconManager->initialize(config);
    if (!success) {
        std::cout << "Falcon Manager failed initialization" << std::endl;
        return 1;
    }

    // Start Tracking
    std::cout << "Start Tracking" << std::endl;
    falconManager->start();
    
    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    falconManager->stop();
    std::cout << "Done" << std::endl;

    return 0;
}