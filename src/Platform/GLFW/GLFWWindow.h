#include "Window.h"

#include <GLFW/glfw3.h>

#include <string>

namespace aimpoint {

	class Window_glfw : public Window {

    public:
        Window_glfw(std::string title, unsigned int height, unsigned int width);
        virtual ~Window_glfw();

        void Update() override;

        unsigned int GetWidth() const override { return m_data.Width; }
        unsigned int GetHeight() const override { return m_data.Height; }
        unsigned int GetXpos() const override { return m_data.Xpos; }
        unsigned int GetYpos() const override { return m_data.Ypos; }

        void SetEventCallback(const EventCallbackFcn& callback) override { m_data.EventCallback = callback; }
        void SetVSync(bool enabled) override;
        bool IsVSync() const override;

        bool ShouldClose() const;

        inline virtual void* GetNativeWindow() const override { return (void*)m_glfwWindow; }
        inline virtual GraphicsContext* GetGraphicsContext() const override { return m_Context; }

    private:
        /* Startup and shutdown functions */
        virtual void Init(std::string title, unsigned int height, unsigned int width);
        virtual void Shutdown();

        /* glfw handle */
        GLFWwindow* m_glfwWindow;
        GraphicsContext* m_Context;

        /* for use with glfw getUserDataPointer */
        struct WindowData {
            std::string Title;
            unsigned int Width, Height;
            unsigned int Xpos, Ypos;
            bool VSync;

            EventCallbackFcn EventCallback;
        };

        WindowData m_data;
    };
}