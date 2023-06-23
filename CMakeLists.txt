cmake_minimum_required( VERSION 3.20 )

set (CMAKE_CXX_STANDARD 17)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "cmake")

set(AppName "aimpoint")

# Overall project
project( ${AppName} )

# set output dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

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

# math library
add_subdirectory("deps/math_lib")

# direct-to-video
option(USE_DTV "Use Direct-To-Video library to save video" OFF) #OFF by default
if(USE_DTV)
    add_subdirectory("deps/DTV")
endif(USE_DTV)


###### Application
if( MSVC )
    SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ENTRY:mainCRTStartup" )
endif()

set(SRC_DIR "aimpoint")
add_executable( aimpoint 
    ${SRC_DIR}/aimpoint.cpp
    ${SRC_DIR}/log.cpp
    ${SRC_DIR}/physics.cpp
    ${SRC_DIR}/rocket.cpp
    ${SRC_DIR}/mesh.cpp
    ${SRC_DIR}/mass_spring_damper.cpp

    ${SRC_DIR}/aimpoint.h
    ${SRC_DIR}/log.h
    ${SRC_DIR}/defines.h
    ${SRC_DIR}/physics.h
    ${SRC_DIR}/mesh.h
    ${SRC_DIR}/mass_spring_damper.h
)

if(USE_DTV)
    target_link_libraries( aimpoint ${OPENGL_LIBRARIES} "${SolverName}" imgui implot spdlog::spdlog laml direct-to-video)
else()
    target_link_libraries( aimpoint ${OPENGL_LIBRARIES} "${SolverName}" imgui implot spdlog::spdlog laml)
endif(USE_DTV)

target_include_directories( aimpoint PRIVATE aimpoint "${CMAKE_SOURCE_DIR}/deps/DTV/include")
if( MSVC )
    set_target_properties(
        ${AppName} PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

    if(${CMAKE_VERSION} VERSION_LESS "3.6.0") 
        message( "\n\t[ WARNING ]\n\n\tCMake version lower than 3.6.\n\n\t - Please update CMake and rerun; OR\n\t - Manually set 'aimpoint' as StartUp Project in Visual Studio.\n" )
    else()
        set_property( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT aimpoint )
    endif()
endif()


unset(USE_DTV CACHE) # <---- this is the important!!