#include <iostream>
#include <memory>
#include <future>
#include <thread>
#include <mavsdk/mavsdk.h>
#include "libs/falcon/include/SdsFalcon.h"
#include "DroneSixdofHover.h"

using namespace mavsdk;
using namespace Sds::Falcon;

std::shared_ptr<mavsdk::System> get_system(Mavsdk& mavsdk){
    auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto fut = prom.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    while(true){
        if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
            std::cerr << "No autopilot found." << std::endl;
        } else {
            break;
        }
    }

    return fut.get();
}

/*
    In this example we show how to do a precise hover and landing using Sixdof tracking.
*/
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

    Config6Dof config;
    
    // add map of beacons - change to your specific map
    Beacon b1 {0.275, 0.00, 0.28, 1408};
    Beacon b2 {0.085, 0.00, -0.035, 1409};
    Beacon b3 {0.01, 0.00, -0.11, 1415};
    Beacon b4 {-0.28, 0.00, -0.29, 1416};
    Beacon b5 {-0.13, 0.00, 0.065, 1421};
    Beacon b6 {-0.28, 0.00, 0.28, 1422};    

    config.map.push_back(b1);
    config.map.push_back(b2);
    config.map.push_back(b3);
    config.map.push_back(b4);
    config.map.push_back(b5);
    config.map.push_back(b6);
    
    bool initSuccess = FalconManager->initialize6Dof(config);
    if (!initSuccess) {
        std::cout << "Drone Manager init Failed" << std::endl;
        return 1;
    }

    /* Connect to drone */
    Mavsdk mavsdk;
	std::cout << "Right before connection string"<< std::endl;
    std::string connection_string; // Where do I use this   
    if (argc == 1 ){
        connection_string = "serial:///dev/serial0:57600";
    } else {
        connection_string = argv[1];
    }

    std::cout << "Connecting to vehicle on" << connection_string << std::endl; 
    ConnectionResult connection_result = mavsdk.add_any_connection(connection_string);
    
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

	for (auto system : mavsdk.systems()) {
	    std::cout << "Found system with MAVLink system ID: " << static_cast<int>(system->get_system_id())
              << ", connected: " << (system->is_connected() ? "yes" : "no")
              << ", has autopilot: " << (system->has_autopilot() ? "yes" : "no") << '\n';
	}

    // Wait for the system to connect via heartbeat
    while (mavsdk.systems().size() == 0) {
        std::cout  << "Looking for system" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // System got discovered.
    std::shared_ptr<System> system = mavsdk.systems()[0];

	std::cout << "Printing Systems" << std::endl;
    for (auto elem : mavsdk.systems()) {
        std::cout << elem << std::endl;
    }

    if (system == nullptr) {
	    std::cout << "System faild to initialize" << std::endl;
        return 1;
    }

    DroneSixdofHover program(system, *FalconManager);
    program.run();

    std::cout << "Execution complete" << std::endl;
    return 0;
}
