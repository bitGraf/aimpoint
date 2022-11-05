#include "GLFWWindow.h"

#include <assert.h>

#include "Platform/OpenGL/OpenGLGraphicsContext.h"
#include "EventTypes.h"

namespace aimpoint {
    /* only init glfw once */
    static bool GLFW_Initialized = false;

    static void GLFWErrorCallback(int error, const char* desc) {
        //ENGINE_LOG_ERROR("GLFW Error ({0}): {1}", error, desc);
        std::cout << "GLFW Error (" << error << "): " << desc << std::endl;
    }

    Window* Window::Create(std::string title, unsigned int height, unsigned int width) {
        return new Window_glfw(title, height, width);
    }

    Window_glfw::Window_glfw(std::string title, unsigned int height, unsigned int width) {
        Init(title, height, width);
    }

    Window_glfw::~Window_glfw() {
        Shutdown();
    }

    bool Window_glfw::ShouldClose() const {
        return glfwWindowShouldClose(m_glfwWindow);
    }

    void Window_glfw::Init(std::string title, unsigned int height, unsigned int width) {
        m_data.Title = title;
        m_data.Height = height;
        m_data.Width = width;

        //ENGINE_LOG_INFO("Creating window {0} ({1}x{2})", title, width, height);
        std::cout << "Creating window " << title << " (" << width << "x" << height << ")" << std::endl;

        /* first time init */
        if (!GLFW_Initialized) {
            // TODO: glfwTerminate on system shutdown
            int success = glfwInit();
            //ENGINE_LOG_ASSERT(success, "Failed to initialize GLFW");
            assert(success);
            glfwSetErrorCallback(GLFWErrorCallback);

            GLFW_Initialized = true;
        }

        m_glfwWindow = glfwCreateWindow((int)width, (int)height, title.c_str(), nullptr, nullptr);
        int xx, yy;
        glfwGetWindowPos(m_glfwWindow, &xx, &yy);
        m_data.Xpos = xx;
        m_data.Ypos = yy;

        m_Context = new OpenGLGraphicsContext(m_glfwWindow);
        m_Context->Init();

        glfwSetWindowUserPointer(m_glfwWindow, &m_data);
        SetVSync(true);

        /* create GLFW callbacks */
        glfwSetWindowSizeCallback(m_glfwWindow, [](GLFWwindow* window, int width, int height) {
            WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);
            data.Width = width;
            data.Height = height;

            WindowResizeEvent event(width, height);
            data.EventCallback(event);
            });

        glfwSetWindowPosCallback(m_glfwWindow, [](GLFWwindow* window, int xpos, int ypos) {
            WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);
            data.Xpos = xpos;
            data.Ypos = ypos;

            WindowMoveEvent event(xpos, ypos);
            data.EventCallback(event);
            });


        glfwSetWindowCloseCallback(m_glfwWindow, [](GLFWwindow* window) {
            WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);

            WindowCloseEvent event;
            data.EventCallback(event);
            });

        glfwSetKeyCallback(m_glfwWindow, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
            WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);

            switch (action)
            {
            case GLFW_PRESS: {
                KeyPressedEvent event(key, 0);
                data.EventCallback(event);
                break;
            }
            case GLFW_RELEASE: {
                KeyReleasedEvent event(key);
                data.EventCallback(event);
                break;
            }
            case GLFW_REPEAT: {
                KeyPressedEvent event(key, 1); // TODO: need to determine repeat count better
                data.EventCallback(event);
                break;
            }
            }
            });

        glfwSetCharCallback(m_glfwWindow, [](GLFWwindow* window, unsigned int keycode) {
            WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);

            KeyTypedEvent event(keycode);
            data.EventCallback(event);
            });

        glfwSetMouseButtonCallback(m_glfwWindow, [](GLFWwindow* window, int button, int action, int mods) {
            WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);

            switch (action)
            {
            case GLFW_PRESS: {
                MouseButtonPressedEvent event(button);
                data.EventCallback(event);
                break;
            }
            case GLFW_RELEASE: {
                MouseButtonReleasedEvent event(button);
                data.EventCallback(event);
                break;
            }
            }
            });

        glfwSetScrollCallback(m_glfwWindow, [](GLFWwindow* window, double xoffset, double yoffset) {
            WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);

            MouseScrolledEvent event((float)xoffset, (float)yoffset);
            data.EventCallback(event);
            });

        glfwSetCursorPosCallback(m_glfwWindow, [](GLFWwindow* window, double xpos, double ypos) {
            WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);

            MouseMovedEvent event((float)xpos, (float)ypos);
            data.EventCallback(event);
            });
    }

    void Window_glfw::Shutdown() {
        glfwDestroyWindow(m_glfwWindow);
    }

    void Window_glfw::Update() {
        glfwPollEvents();
        m_Context->SwapBuffers();
    }

    void Window_glfw::SetVSync(bool enabled) {
        glfwSwapInterval(enabled ? 1 : 0);

        m_data.VSync = enabled;
    }

    bool Window_glfw::IsVSync() const {
        return m_data.VSync;
    }
}