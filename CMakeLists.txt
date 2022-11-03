cmake_minimum_required( VERSION 3.2 )

project( aimpoint )

# GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory( "deps/GLFW" )

# imgui
add_subdirectory("deps/imgui")

# glad
add_subdirectory("deps/glad")

# glm
add_subdirectory("deps/glm")

if( MSVC )
    SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ENTRY:mainCRTStartup" )
endif()

set( aimpoint-SRC
     main.cpp
     )
     
add_executable( aimpoint WIN32 ${aimpoint-SRC} )
target_link_libraries( aimpoint ${OPENGL_LIBRARIES} glfw glad imgui glm)
if( MSVC )
    if(${CMAKE_VERSION} VERSION_LESS "3.6.0") 
        message( "\n\t[ WARNING ]\n\n\tCMake version lower than 3.6.\n\n\t - Please update CMake and rerun; OR\n\t - Manually set 'aimpoint' as StartUp Project in Visual Studio.\n" )
    else()
        set_property( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT aimpoint )
    endif()
endif()

