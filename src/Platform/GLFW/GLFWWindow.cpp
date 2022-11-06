#include "GLFWWindow.h"

#include "Core/EventTypes.h"

#include <imgui.h>

namespace aimpoint {

    /* only init glfw once */
    static bool GLFW_Initialized = false;

    static void GLFWErrorCallback(int error, const char* desc) {
        LOG_ERROR("GLFW Error ({0}): {1}", error, desc);
    }

    Window* Window::Create(const WindowProps& props) {
        return new Window_glfw(props);
    }

    Window_glfw::Window_glfw(const WindowProps& props) {
        Init(props);
    }

    Window_glfw::~Window_glfw() {
    }

    void Window_glfw::Init(const WindowProps& props) {
        m_data.Title = props.Title;
        m_data.Height = props.Height;
        m_data.Width = props.Width;

        LOG_INFO("Creating window {0} ({1}x{2})", props.Title, props.Height, props.Width);

        /* first time init */
        if (!GLFW_Initialized) {
            // TODO: glfwTerminate on system shutdown
            int success = glfwInit();
            LOG_ASSERT(success, "Failed to initialize GLFW");
            glfwSetErrorCallback(GLFWErrorCallback);

            GLFW_Initialized = true;
        }

        //if (RendererAPI::Current() == RendererAPIType::Vulkan)
        //    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        m_glfwWindow = glfwCreateWindow((int)props.Width, (int)props.Height, props.Title.c_str(), nullptr, nullptr);

        m_RenderContext = RendererContext::Create(m_glfwWindow);
        //m_Context->Create();

        //glfwMaximizeWindow(m_Window);
        glfwSetWindowUserPointer(m_glfwWindow, &m_data);

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

        m_ImGuiMouseCursors[ImGuiMouseCursor_Arrow] = glfwCreateStandardCursor(GLFW_ARROW_CURSOR);
        m_ImGuiMouseCursors[ImGuiMouseCursor_TextInput] = glfwCreateStandardCursor(GLFW_IBEAM_CURSOR);
        m_ImGuiMouseCursors[ImGuiMouseCursor_ResizeAll] = glfwCreateStandardCursor(GLFW_ARROW_CURSOR);   // FIXME: GLFW doesn't have this.
        m_ImGuiMouseCursors[ImGuiMouseCursor_ResizeNS] = glfwCreateStandardCursor(GLFW_VRESIZE_CURSOR);
        m_ImGuiMouseCursors[ImGuiMouseCursor_ResizeEW] = glfwCreateStandardCursor(GLFW_HRESIZE_CURSOR);
        m_ImGuiMouseCursors[ImGuiMouseCursor_ResizeNESW] = glfwCreateStandardCursor(GLFW_ARROW_CURSOR);  // FIXME: GLFW doesn't have this.
        m_ImGuiMouseCursors[ImGuiMouseCursor_ResizeNWSE] = glfwCreateStandardCursor(GLFW_ARROW_CURSOR);  // FIXME: GLFW doesn't have this.
        m_ImGuiMouseCursors[ImGuiMouseCursor_Hand] = glfwCreateStandardCursor(GLFW_HAND_CURSOR);

        // Update window size to actual size
        {
            int width, height;
            glfwGetWindowSize(m_glfwWindow, &width, &height);
            m_data.Width = width;
            m_data.Height = height;
        }
    }

    void Window_glfw::Shutdown() {
    }

    inline std::pair<float, float> Window_glfw::GetWindowPos() const
    {
        int x, y;
        glfwGetWindowPos(m_glfwWindow, &x, &y);
        return { x, y };
    }

    void Window_glfw::ProcessEvents() {
        glfwPollEvents();

        //ImGuiMouseCursor imgui_cursor = ImGui::GetMouseCursor();
        //glfwSetCursor(m_Window, m_ImGuiMouseCursors[imgui_cursor] ? m_ImGuiMouseCursors[imgui_cursor] : m_ImGuiMouseCursors[ImGuiMouseCursor_Arrow]);
        glfwSetInputMode(m_glfwWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }

    void Window_glfw::SwapBuffers()
    {
        m_RenderContext->SwapBuffers();
    }

    void Window_glfw::SetVSync(bool enabled) {
        if (true /*RendererAPI::Current() == RendererAPIType::OpenGL*/)
        {
            if (enabled)
                glfwSwapInterval(1);
            else
                glfwSwapInterval(0);
        }

        m_data.VSync = enabled;
    }

    bool Window_glfw::IsVSync() const {
        return m_data.VSync;
    }

    void Window_glfw::SetTitle(const std::string& title)
    {
        m_data.Title = title;
        glfwSetWindowTitle(m_glfwWindow, m_data.Title.c_str());
    }
}