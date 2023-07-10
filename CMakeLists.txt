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

# math library
add_subdirectory("deps/stb")

# direct-to-video
option(USE_DTV_LIB "Include Direct-To-Video library to save video" OFF) #OFF by default
option(USE_DTV "Enable DTV in the code" OFF) #OFF by default
if(USE_DTV_LIB)
    add_subdirectory("deps/DTV")
    if(USE_DTV)
        add_compile_definitions("USE_DTV=1")
    else()
        add_compile_definitions("USE_DTV=0")
    endif()
endif(USE_DTV_LIB)


###### Aimpoint-lib
if( MSVC )
    SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ENTRY:mainCRTStartup" )
endif()

set(SRC_DIR "aimpoint")
add_library( aimpoint-lib STATIC
    ${SRC_DIR}/base_app.cpp
    ${SRC_DIR}/log.cpp
    ${SRC_DIR}/physics.cpp
    ${SRC_DIR}/planet.cpp
    ${SRC_DIR}/orbit.cpp
    ${SRC_DIR}/body_type/t_bar.cpp
    ${SRC_DIR}/body_type/mass_spring_damper.cpp
    ${SRC_DIR}/body_type/satellite.cpp
    ${SRC_DIR}/render/renderer.cpp
    ${SRC_DIR}/render/mesh.cpp
    ${SRC_DIR}/render/texture.cpp
    ${SRC_DIR}/render/shader_program.cpp

    ${SRC_DIR}/base_app.h
    ${SRC_DIR}/log.h
    ${SRC_DIR}/defines.h
    ${SRC_DIR}/physics.h
    ${SRC_DIR}/planet.h
    ${SRC_DIR}/orbit.h
    ${SRC_DIR}/body_type/t_bar.h
    ${SRC_DIR}/body_type/mass_spring_damper.h
    ${SRC_DIR}/body_type/satellite.h
    ${SRC_DIR}/render/renderer.h
    ${SRC_DIR}/render/mesh.h
    ${SRC_DIR}/render/texture.h
    ${SRC_DIR}/render/shader_program.h
)

if(USE_DTV_LIB)
    target_link_libraries( aimpoint-lib ${OPENGL_LIBRARIES} imgui implot spdlog::spdlog stb laml direct-to-video)
else()
    target_link_libraries( aimpoint-lib ${OPENGL_LIBRARIES} imgui implot spdlog::spdlog stb laml)
endif(USE_DTV_LIB)

target_include_directories( aimpoint-lib PUBLIC aimpoint "${CMAKE_SOURCE_DIR}/deps/DTV/include")

####### Aimpoint
add_executable(aimpoint
    ${SRC_DIR}/aimpoint.cpp
    ${SRC_DIR}/aimpoint.h
    )
target_link_libraries( aimpoint ${OPENGL_LIBRARIES} aimpoint-lib)

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

option(INCLUDE_DEMOS "Include projects for various demos" ON) #ON by default
if(INCLUDE_DEMOS)
    add_subdirectory("demos")
endif(INCLUDE_DEMOS)

unset(INCLUDE_DEMOS CACHE) # <---- this is the important!!
unset(USE_DTV_LIB CACHE) # <---- this is the important!!
unset(USE_DTV CACHE) # <---- this is the important!!