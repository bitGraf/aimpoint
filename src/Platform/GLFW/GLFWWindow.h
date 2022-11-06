#include "Core/Window.h"

#include <GLFW/glfw3.h>

#include <string>

namespace aimpoint {

	class Window_glfw : public Window {

    public:
        Window_glfw(const WindowProps& props);
        virtual ~Window_glfw();

        virtual void ProcessEvents() override;
        virtual void SwapBuffers() override;

        inline uint32_t GetWidth() const override { return m_data.Width; }
        inline uint32_t GetHeight() const override { return m_data.Height; }
        
        virtual std::pair<uint32_t, uint32_t> GetSize() const override { return { m_data.Width, m_data.Height }; }
        virtual std::pair<float, float> GetWindowPos() const override;

        void SetEventCallback(const EventCallbackFcn& callback) override { m_data.EventCallback = callback; }
        void SetVSync(bool enabled) override;
        bool IsVSync() const override;

        virtual const std::string& GetTitle() const override { return m_data.Title; }
        virtual void SetTitle(const std::string& title) override;

        inline virtual void* GetNativeWindow() const override { return (void*)m_glfwWindow; }

        inline virtual RendererContext* GetRenderContext() override { return m_RenderContext; }

    private:
        /* Startup and shutdown functions */
        virtual void Init(const WindowProps& props);
        virtual void Shutdown();

        /* glfw handle */
        GLFWwindow* m_glfwWindow;
        GLFWcursor* m_ImGuiMouseCursors[9] = { 0 };

        /* for use with glfw getUserDataPointer */
        struct WindowData {
            std::string Title;
            unsigned int Width, Height;
            unsigned int Xpos, Ypos;
            bool VSync;

            EventCallbackFcn EventCallback;
        };

        WindowData m_data;
        float m_LastFrameTime = 0.0f;

        RendererContext* m_RenderContext;
    };
}