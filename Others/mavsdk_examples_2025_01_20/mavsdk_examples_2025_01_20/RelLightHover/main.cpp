#include <iostream>
#include <memory>
#include <future>
#include <mavsdk/mavsdk.h>
#include "libs/falcon/include/SdsFalcon.h"
#include "RelLightHover.h"

using namespace mavsdk;
using namespace Sds::Falcon;


std::shared_ptr<System> get_system(Mavsdk& mavsdk) // Note: acquired from libs/MAVSDK-1.4/examples/takeoff_and_land/takeoff_and_land.cpp
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}


int main(int argc, char** argv){
    
    /* Initalize Sixdof Sensor */
    std::unique_ptr<FalconManager> FalconManager = createFalconManager();
    
    FalconManager->registerMessageCallback([](const StatusMessage& msg) {
        std::cout << msg.message << std::endl;
    });
    
    NetworkConfig network_config { 30, "eth0" }; // cahnge to your sensor_id and network

    bool initNetworkSuccess = FalconManager->initializeNetwork(network_config);
    if (!initNetworkSuccess) {
        std::cout << "Failed to initalize network" << std::endl;
        return 1;
    }

    ConfigRelativeBeacon config; // use defaults

    bool initSuccess = FalconManager->initializeRelativeBeacon(config);
    if (!initSuccess) {
        std::cout << "Drone Manager init Failed" << std::endl;
        return 1;
    }

    std::string connection_string; // Where do I use this   
    if(argv == nullptr){
        std::string connection_string = "serial:///dev/serial0:57600";
    }else{
        std::string connection_string = argv[0]; //Is this right?
    }

    // Connecting to drone
    Mavsdk mavsdk;
    std::cout << "Connecting to vehicle on" << connection_string << std::endl; 
    ConnectionResult connection_result = mavsdk.add_any_connection(connection_string);
    
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    // Creating Drone
    auto system = get_system(mavsdk);
    if (!system) {
        return 1;
    }

    RelLightHover program(*system, *FalconManager);
    program.run();

    return 0;
}