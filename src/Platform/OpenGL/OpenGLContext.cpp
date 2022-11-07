#include "OpenGLContext.h"

#include <glad/gl.h>
#include <glfw/glfw3.h>

namespace aimpoint {

    OpenGLContext::OpenGLContext(GLFWwindow* windowHandle)
        : m_WindowHandle(windowHandle) {
        LOG_ASSERT(windowHandle, "Window handle does not exist");
    }

    OpenGLContext::~OpenGLContext() {
    }

    void OpenGLContext::Create() {
        LOG_INFO("Creating OpenGLContext");
        glfwMakeContextCurrent(m_WindowHandle);
        int status = gladLoadGL((GLADloadfunc)glfwGetProcAddress);
        LOG_ASSERT(status, "Failed to initialize GLAD");

        LOG_INFO("OpenGL Info:");
        LOG_INFO("  Vendor:   {0}", (char*)glGetString(GL_VENDOR));
        LOG_INFO("  Renderer: {0}", (char*)glGetString(GL_RENDERER));
        LOG_INFO("  Version:  {0}", (char*)glGetString(GL_VERSION));
        LOG_INFO("  Version:  {0}", (char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

        int versionMajor, versionMinor;
        glGetIntegerv(GL_MAJOR_VERSION, &versionMajor);
        glGetIntegerv(GL_MINOR_VERSION, &versionMinor);

        LOG_ASSERT(versionMajor > 4 || (versionMajor == 4 && versionMinor >= 6), "OpenGL version needs to be at least 4.6!");
    }

    void OpenGLContext::SwapBuffers() {
        glfwSwapBuffers(m_WindowHandle);
    }

    void OpenGLContext::BeginFrame() {
        glClear(GL_COLOR_BUFFER_BIT);
    }
}