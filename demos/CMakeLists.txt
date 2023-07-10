add_executable( spinning_rigid_body 
    spinning_rigid_body.cpp
    spinning_rigid_body.h
)
target_include_directories(spinning_rigid_body PUBLIC "../aimpoint")
target_link_libraries(spinning_rigid_body PUBLIC aimpoint-lib)
set_property(TARGET spinning_rigid_body PROPERTY FOLDER "Demos")


add_executable( flat_earth_launch 
    flat_earth_launch.h
    flat_earth_launch.cpp

    flat_earth_rocket.h
    flat_earth_rocket.cpp
)
target_include_directories(flat_earth_launch PUBLIC "../aimpoint")
target_link_libraries(flat_earth_launch PUBLIC aimpoint-lib)
set_property(TARGET flat_earth_launch PROPERTY FOLDER "Demos")


if( MSVC )
    set_target_properties(
        spinning_rigid_body PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
    set_target_properties(
        flat_earth_launch PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif()