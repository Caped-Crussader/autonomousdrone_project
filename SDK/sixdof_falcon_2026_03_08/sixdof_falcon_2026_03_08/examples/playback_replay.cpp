#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include <atomic>
#include "SdsFalcon.h"

using namespace Sds::Falcon;


// ***************************************************************************
// *                                                                         *
// *                     DataPlayback Replay Example                         *
// *                                                                         *
// *   This example demonstrates how to replay previously recorded data.     *
// *                                                                         *
// ***************************************************************************


int main(int ac, const char *av[]) {

    // Print the SdsFalcon SDK version
    Version version = getVersion();
    std::cout << "Using SdsFalcon version: " << version.getString() << std::endl;

    // Initialize the Falcon Manager
    std::unique_ptr<FalconManager> falconManager = createFalconManager();
    std::cout << "Falcon Manager created" << std::endl;

    // Track if reached End Of File (EOF)
    std::atomic<bool> reached_eof { false };

    // Register callbacks before calling the init function so we can get error messages printed out
    falconManager->registerMessageCallback([&](const StatusMessage& msg){
        std::cout << msg.message << std::endl;
        if (msg.layer == CallingLayer::SdsFalcon && msg.event_code == (uint8_t)FalconEventCode::PlaybackReachedEndOfFile) {
            reached_eof.store(true);
        }
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

    // config can be empty, all required information is in the recording
    Config config;

    // Playback Settings
    // Input the path for the recording file
    DataPlaybackSettings playbackSettings { DataPlaybackMode::PLAYBACK, "C:\\example_file.bin" };

    // Initialize
    bool success = falconManager->initialize(config, playbackSettings);
    if (!success) {
        std::cout << "Falcon Manager failed initialization" << std::endl;
        return 1;
    }

    // Start Tracking
    std::cout << "Start Tracking" << std::endl;
    falconManager->start();
    
    // wait until end of file is reached
    while(!reached_eof.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    
    falconManager->stop();
    std::cout << "Done" << std::endl;

    return 0;
}