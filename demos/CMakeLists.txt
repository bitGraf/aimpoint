add_executable( spinning_rigid_body 
    spinning_rigid_body.cpp
    spinning_rigid_body.h
)
target_include_directories(spinning_rigid_body PUBLIC "../aimpoint")
target_link_libraries(spinning_rigid_body PUBLIC aimpoint-lib)

#set_property(TARGET spinning_rigid_body PROPERTY USE_FOLDERS ON)
set_property(TARGET spinning_rigid_body PROPERTY FOLDER "Demos")