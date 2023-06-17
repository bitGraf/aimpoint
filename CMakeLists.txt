cmake_minimum_required( VERSION 3.20 )

set (CMAKE_CXX_STANDARD 17)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "cmake")

set(AppName "aimpoint")

# Overall project
project( ${AppName} )

# GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory( "deps/GLFW" )

# imgui/implot
add_subdirectory("deps/imgui")
add_subdirectory("deps/implot")

# glad
add_subdirectory("deps/glad")

# spdlog
add_subdirectory("deps/spdlog")


###### Application
if( MSVC )
    SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ENTRY:mainCRTStartup" )
endif()

set(SRC_DIR "aimpoint")
add_executable( aimpoint 
    ${SRC_DIR}/aimpoint.cpp
    ${SRC_DIR}/log.cpp
    ${SRC_DIR}/physics.cpp

    ${SRC_DIR}/aimpoint.h
    ${SRC_DIR}/log.h
    ${SRC_DIR}/defines.h
    ${SRC_DIR}/physics.h
)

target_link_libraries( aimpoint ${OPENGL_LIBRARIES} "${SolverName}" imgui implot spdlog::spdlog)
target_include_directories( aimpoint PRIVATE aimpoint)
if( MSVC )
    if(${CMAKE_VERSION} VERSION_LESS "3.6.0") 
        message( "\n\t[ WARNING ]\n\n\tCMake version lower than 3.6.\n\n\t - Please update CMake and rerun; OR\n\t - Manually set 'aimpoint' as StartUp Project in Visual Studio.\n" )
    else()
        set_property( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT aimpoint )
    endif()
endif()