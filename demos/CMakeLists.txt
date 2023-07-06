add_executable( spinning_rigid_body 
    spinning_rigid_body.cpp
    spinning_rigid_body.h
)
target_include_directories(spinning_rigid_body PUBLIC "../aimpoint")
target_link_libraries(spinning_rigid_body PUBLIC aimpoint-lib)

set_property(TARGET spinning_rigid_body PROPERTY FOLDER "Demos")
if( MSVC )
    set_target_properties(
        spinning_rigid_body PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif()