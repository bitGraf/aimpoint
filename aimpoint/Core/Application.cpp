#include "Application.h"

#include "KeyCodes.h"

#include <glfw/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "implot.h"

#define BIND_EVENT_FN(fn) [this](auto&&... args) -> decltype(auto) { return this->fn(std::forward<decltype(args)>(args)...); }

namespace aimpoint {

	Application::Application() : m_done(false), m_minimized(false), m_window(nullptr) {
	}

	Application::~Application() {
	}

	void Application::Run() {
        aimpoint::Log::Init();
        LOG_INFO("Hiyaa =^.^=");

        // Simulations params

        m_clock.Create(1/120.0, 0.0);

        aimpoint::Window* window = aimpoint::Window::Create();
        window->SetEventCallback(BIND_EVENT_FN(Application::HandleEvent));

        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;
        //io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

        ImPlot::CreateContext();

        // Setup Dear ImGui style
        ImGui::StyleColorsDark();
        //ImGui::StyleColorsLight();

        // Setup Platform/Renderer backends
        ImGui_ImplGlfw_InitForOpenGL((GLFWwindow*)window->GetNativeWindow(), true);
        ImGui_ImplOpenGL3_Init("#version 460");

        // Our state
        bool show_demo_window = true;
        bool show_another_window = false;
        ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

        while (!m_done) {
            //LOG_TRACE("t={:08.3f}x={:18.4f}v={:28.4f}", m_clock.GetTime(), x.x, x.y);

            window->GetRenderContext()->BeginFrame(); //TODO: temp workaround

            // Start the Dear ImGui frame
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();    

            // Rendering
            ImGui::Render();

            // Update and Render additional Platform Windows
            if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
            {
                GLFWwindow* backup_current_context = glfwGetCurrentContext();
                ImGui::UpdatePlatformWindows();
                ImGui::RenderPlatformWindowsDefault();
                glfwMakeContextCurrent(backup_current_context);
            }
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            window->SwapBuffers();
            m_clock.Advance();
            window->ProcessEvents();
        }

        ImPlot::DestroyContext();
        ImGui::DestroyContext();

        LOG_INFO("Shutting Down...");
	}

    void Application::Close() {
        m_done = true;
    }

    void Application::HandleEvent(aimpoint::Event& event) {
        aimpoint::EventDispatcher dispatcher(event);
        dispatcher.Dispatch<aimpoint::KeyPressedEvent>(BIND_EVENT_FN(Application::OnKeyPressedEvent));
        dispatcher.Dispatch<aimpoint::WindowCloseEvent>(BIND_EVENT_FN(Application::OnWindowClose));
        dispatcher.Dispatch<aimpoint::WindowResizeEvent>(BIND_EVENT_FN(Application::OnWindowResize));
    }

    bool Application::OnKeyPressedEvent(KeyPressedEvent& event) {
        if(event.GetKeyCode() == KEY_CODE_ESCAPE) {
            Close();
        }

        return false;
    }

    bool Application::OnWindowClose(WindowCloseEvent& event) {
        m_done = true;
        return true;
    }

    bool Application::OnWindowResize(WindowResizeEvent& event) {
        if (event.GetWidth() == 0 || event.GetHeight() == 0) {
            m_minimized = true;
            return false;
        }

        m_minimized = false;
        //Renderer::OnWindowResize(event.GetWidth(), event.GetHeight());

        return false;
    }
}