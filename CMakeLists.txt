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

# spdlog
add_subdirectory("deps/spdlog")

if( MSVC )
    SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ENTRY:mainCRTStartup" )
endif()

set( aimpoint-CORE-SRC
     src/Core.h
     src/main.cpp
     src/Log.h
     src/Log.cpp
     src/Application.h
     src/Application.cpp
     src/Window.h
     src/Event.h
     src/EventTypes.h
     src/Platform/GLFW/GLFWWindow.cpp
     src/Platform/GLFW/GLFWWindow.h
     )

set( aimpoint-RENDERER-SRC 
     src/Renderer/RendererContext.h
     src/Renderer/RendererContext.cpp
     src/Renderer/Framebuffer.h
    )

set( aimpoint-OPENGL-SRC
     src/Platform/OpenGL/OpenGLContext.cpp
     src/Platform/OpenGL/OpenGLContext.h
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
add_executable( aimpoint ${aimpoint-CORE-SRC} ${aimpoint-RENDERER-SRC} ${aimpoint-OPENGL-SRC})
source_group("Core"     FILES ${aimpoint-CORE-SRC} )
source_group("Renderer" FILES ${aimpoint-RENDERER-SRC} )
source_group("OpenGL"   FILES ${aimpoint-OPENGL-SRC} )

target_link_libraries( aimpoint ${OPENGL_LIBRARIES} glfw glad imgui glm spdlog::spdlog)
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