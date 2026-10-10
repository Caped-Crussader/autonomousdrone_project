
add_executable(takeoff_and_land_demo
               DroneControl.cpp
               main.cpp
)



target_link_libraries(takeoff_and_land_demo mavsdk SdsFalcon)

