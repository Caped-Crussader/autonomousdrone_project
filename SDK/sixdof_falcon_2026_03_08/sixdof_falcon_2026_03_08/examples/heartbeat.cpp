#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsFalcon.h"

using namespace Sds::Falcon;


// ***************************************************************************
// *                                                                         *
// *                      Heartbeat Feature Example #1                       *
// *                                                                         *
// *   This example demonstrates how to use the Heartbeat feature.           *
// *   The Heartbeat is a callback which shows that the communication        *
// *   between the computer and the sensor is working, and that the runnning *
// *   state of the Falcon Manager is valid.                                 * 
// *                                                                         *
// ***************************************************************************


int main(int ac, const char *av[]) {
    
    // Print the SdsFalcon SDK version
    Version version = getVersion();
    std::cout << "Using SdsFalcon version: " << version.getString() << std::endl;

    // Initialize the Falcon Manager
    std::unique_ptr<FalconManager> falconManager = createFalconManager();
    std::cout << "Falcon Manager created" << std::endl;

    // Register callback before calling the init function so we can get the heartbeat
    falconManager->registerHeartbeatCallback([](const Heartbeat& h){
            std::cout << " comm " << (h.sensor_communication_ok ? "ON" : "OFF")
                      << " state " << to_string(h.current_tracking_state) << std::endl;
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