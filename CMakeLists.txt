cmake_minimum_required( VERSION 3.20 )

set (CMAKE_CXX_STANDARD 17)

project( aimpoint )

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

# glm
add_subdirectory("deps/glm")

# spdlog
add_subdirectory("deps/spdlog")

if( MSVC )
    SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ENTRY:mainCRTStartup" )
endif()

set( SRC-DIR "src")
set( CORE-SRC-DIR "${SRC-DIR}/Core")
set( OpenGL-SRC-DIR "${SRC-DIR}/Platform/OpenGL")
set( Renderer-SRC-DIR "${SRC-DIR}/Renderer")

set(aimpoint-CORE-SRC
    ${CORE-SRC-DIR}/Application.cpp
    ${CORE-SRC-DIR}/Application.h
    ${CORE-SRC-DIR}/Core.h
    ${CORE-SRC-DIR}/Event.h
    ${CORE-SRC-DIR}/EventTypes.h
    ${CORE-SRC-DIR}/KeyCodes.h
    ${CORE-SRC-DIR}/Log.cpp
    ${CORE-SRC-DIR}/Log.h
    ${CORE-SRC-DIR}/Window.h

    ${SRC-DIR}/Math/RK4.cpp
    ${SRC-DIR}/Math/RK4.h
    ${SRC-DIR}/Math/SignalLogger.h

    ${CORE-SRC-DIR}/Timing/Clock.h
    ${CORE-SRC-DIR}/Timing/Clock.cpp
    
    ${SRC-DIR}/Platform/GLFW/GLFWWindow.cpp
    ${SRC-DIR}/Platform/GLFW/GLFWWindow.h
    )

set( aimpoint-RENDERER-SRC 
     ${Renderer-SRC-DIR}/RendererContext.h
     ${Renderer-SRC-DIR}/RendererContext.cpp
     ${Renderer-SRC-DIR}/Framebuffer.h
    )

set( aimpoint-OPENGL-SRC
     ${OpenGL-SRC-DIR}/OpenGLContext.cpp
     ${OpenGL-SRC-DIR}/OpenGLContext.h
     # src/Platform/OpenGL/OpenGLFramebuffer.cpp
     # src/Platform/OpenGL/OpenGLFramebuffer.h
     # src/Platform/OpenGL/OpenGLImGuiLayer.cpp
     # src/Platform/OpenGL/OpenGLImGuiLayer.h
     # src/Platform/OpenGL/OpenGLIndexBuffer.cpp
     # src/Platform/OpenGL/OpenGLIndexBuffer.h
     # src/Platform/OpenGL/OpenGLMaterial.cpp
     # src/Platform/OpenGL/OpenGLMaterial.h
     # src/Platform/OpenGL/OpenGLPipeline.cpp
     # src/Platform/OpenGL/OpenGLPipeline.h
     # src/Platform/OpenGL/OpenGLRenderer.cpp
     # src/Platform/OpenGL/OpenGLRenderer.h
     # src/Platform/OpenGL/OpenGLRenderPass.cpp
     # src/Platform/OpenGL/OpenGLRenderPass.h
     # src/Platform/OpenGL/OpenGLShader.cpp
     # src/Platform/OpenGL/OpenGLShader.h
     # src/Platform/OpenGL/OpenGLTexture.cpp
     # src/Platform/OpenGL/OpenGLTexture.h
     # src/Platform/OpenGL/OpenGLVertexBuffer.cpp
     # src/Platform/OpenGL/OpenGLVertexBuffer.h
    )
     
# add_executable( aimpoint WIN32 ${aimpoint-SRC} )
add_executable( aimpoint ${SRC-DIR}/main.cpp ${aimpoint-CORE-SRC} ${aimpoint-RENDERER-SRC} ${aimpoint-OPENGL-SRC})
source_group("Core"     FILES ${aimpoint-CORE-SRC} )
source_group("Renderer" FILES ${aimpoint-RENDERER-SRC} )
source_group("OpenGL"   FILES ${aimpoint-OPENGL-SRC} )

target_link_libraries( aimpoint ${OPENGL_LIBRARIES} glfw glad imgui implot glm spdlog::spdlog)
target_include_directories( aimpoint PRIVATE src/)
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