cmake_minimum_required( VERSION 3.20 )

set (CMAKE_CXX_STANDARD 17)

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
     src/main.cpp
     src/window.h
     src/platform/window_impl_glfw.cpp
     src/platform/window_impl_glfw.h
     src/event.h
     src/event_types.h
     )
     
add_executable( aimpoint WIN32 ${aimpoint-SRC} )
target_link_libraries( aimpoint ${OPENGL_LIBRARIES} glfw glad imgui glm)
target_include_directories( aimpoint PRIVATE src/)
target_precompile_headers(aimpoint PRIVATE 
    <string>
    <functional>
    <vector>
    <unordered_map>
    <iostream>
    )
if( MSVC )
    if(${CMAKE_VERSION} VERSION_LESS "3.6.0") 
        message( "\n\t[ WARNING ]\n\n\tCMake version lower than 3.6.\n\n\t - Please update CMake and rerun; OR\n\t - Manually set 'aimpoint' as StartUp Project in Visual Studio.\n" )
    else()
        set_property( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT aimpoint )
    endif()
endif()