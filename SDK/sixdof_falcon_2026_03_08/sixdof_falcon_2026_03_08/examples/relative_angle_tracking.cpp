#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsFalcon.h"

using namespace Sds::Falcon;


// ***************************************************************************
// *                                                                         *
// *                          Tracking Example                               *
// *                                                                         *
// *   This example demonstrates the typical workflow for using the          *
// *   SdsFalcon SDK for tracking.                                           *
// *   It is important to note the order of function calls:                  *
// *   - Create a Falcon Manager                                             *
// *   - Register Callbacks                                                  *
// *   - Create the Configuration                                            *
// *   - Initialize (establishes a connection to the sensor)                 *
// *   - Start the Tracking                                                  *
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

    falconManager->registerRelativeAngleCallback([](const RelativeAngleCollection& col){
        for (const RelativeAngle& ra : col) { 
            std::cout << "ID: " << ra.id
                    << std::fixed << std::setprecision(0)
                    << " XAng: " << std::setw(3) << ra.x_angle * 180.0 / 3.14159
                    << " ZAng: " << std::setw(3) << ra.z_angle * 180.0 / 3.14159 
                    << " Intensity: " << std::setw(4) << ra.intensity 
                    << std::fixed << std::setprecision(2)
                    << " Width: " << std::setw(5) << ra.width << std::endl;
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