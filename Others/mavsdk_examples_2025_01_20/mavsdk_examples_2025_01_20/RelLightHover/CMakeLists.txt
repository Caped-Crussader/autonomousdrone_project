
add_executable(rel_light_hover
               RelLightHover.cpp
               main.cpp
)



target_link_libraries(rel_light_hover mavsdk SdsFalcon)
