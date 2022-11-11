cmake_minimum_required( VERSION 3.20 )

set (CMAKE_CXX_STANDARD 17)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "cmake")

set(SolverName "solver")
set(SolverNamespace "graf_solver")
set(AppName "aimpoint")

# Overall project
project( ${AppName} )

# glm
add_subdirectory("deps/glm")

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

###### Solver library
set(solver_src "solver")
set(solver_inc "solver/include/solver")
add_library("${SolverName}" STATIC
    # Source files
    ${solver_src}/RigidBodySystem.cpp
    ${solver_src}/SystemState.cpp
    ${solver_src}/OdeSolver.cpp
    ${solver_src}/RigidBody.cpp
    ${solver_src}/Utils.cpp

    # Header files
    ${solver_inc}/RigidBodySystem.h
    ${solver_inc}/SystemState.h
    ${solver_inc}/OdeSolver.h
    ${solver_inc}/RigidBody.h
    ${solver_inc}/Utils.h
)
target_link_libraries("${SolverName}" glm)
target_compile_definitions("${SolverName}" PRIVATE PROJECT_NAMESPACE=${SolverNamespace})
target_include_directories("${SolverName}" PRIVATE ${solver_inc})


###### Application
if( MSVC )
    SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ENTRY:mainCRTStartup" )
endif()

set(SRC_DIR "aimpoint")
add_executable( aimpoint 
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/Core/Application.cpp
    ${SRC_DIR}/Core/Log.cpp
    ${SRC_DIR}/Core/Timing/Clock.cpp
    ${SRC_DIR}/Renderer/RendererContext.cpp
    ${SRC_DIR}/Platform/GLFW/GLFWWindow.cpp
    ${SRC_DIR}/Platform/OpenGL/OpenGLContext.cpp

    ${SRC_DIR}/Core/Application.h
    ${SRC_DIR}/Core/Core.h
    ${SRC_DIR}/Core/Event.h
    ${SRC_DIR}/Core/EventTypes.h
    ${SRC_DIR}/Core/KeyCodes.h
    ${SRC_DIR}/Core/Log.h
    ${SRC_DIR}/Core/Window.h
    ${SRC_DIR}/Renderer/RendererContext.h
    ${SRC_DIR}/Platform/GLFW/GLFWWindow.h
    ${SRC_DIR}/Platform/OpenGL/OpenGLContext.h
)

target_link_libraries( aimpoint ${OPENGL_LIBRARIES} "${SolverName}" imgui implot spdlog::spdlog)
target_include_directories( aimpoint PRIVATE solver/include/ aimpoint/)
target_precompile_headers(aimpoint PRIVATE 
    <string>
    <functional>
    <vector>
    <unordered_map>
    <iostream>
    <stdint.h>
    )
if( MSVC )
    if(${CMAKE_VERSION} VERSION_LESS "3.6.0") 
        message( "\n\t[ WARNING ]\n\n\tCMake version lower than 3.6.\n\n\t - Please update CMake and rerun; OR\n\t - Manually set 'aimpoint' as StartUp Project in Visual Studio.\n" )
    else()
        set_property( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT aimpoint )
    endif()
endif()