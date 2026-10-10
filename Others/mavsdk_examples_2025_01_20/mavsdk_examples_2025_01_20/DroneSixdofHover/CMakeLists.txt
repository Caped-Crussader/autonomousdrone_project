
add_executable(drone_hover_sixdof
               DroneSixdofHover.cpp
               main.cpp
)



target_link_libraries(drone_hover_sixdof mavsdk SdsFalcon)

