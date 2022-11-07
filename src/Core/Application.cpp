#include "Application.h"

#include "KeyCodes.h"

#include "Math/RK4.h"
#include "Math/SignalLogger.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#define BIND_EVENT_FN(fn) [this](auto&&... args) -> decltype(auto) { return this->fn(std::forward<decltype(args)>(args)...); }

// system parameters
float b = 0.1f;
float M = 1.0f;
float K = 1.0f;
float L = 0.0f;
glm::vec2 mass_spring_damper(float t, glm::vec2 x) {
    glm::vec2 x_dot;
    x_dot.x = x.y; // x1_dot = x2
    x_dot.y = -(b/M)*x.y - (K/M)*x.x + K*L/M;
    return x_dot;
}

namespace aimpoint {

	Application::Application() : m_done(false), m_minimized(false), m_window(nullptr) {
	}

	Application::~Application() {
	}

	void Application::Run() {
        aimpoint::Log::Init();
        LOG_INFO("Hiyaa =^.^=");

        m_clock.Create(0.1, 0.0);

        aimpoint::Window* window = aimpoint::Window::Create();
        window->SetEventCallback(BIND_EVENT_FN(Application::HandleEvent));

        // Initialize mass-spring-damper system
        glm::vec2 x(1.5, 0);
        SignalLogger<250> logger_x1;
        SignalLogger<250> logger_x2;

        logger_x1.AddSample(x.x);
        logger_x2.AddSample(x.y);

        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;

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
            x = Integrate_RK4(mass_spring_damper, x, m_clock);
            logger_x1.AddSample(x.x);
            logger_x2.AddSample(x.y);

            window->GetRenderContext()->BeginFrame(); //TODO: temp workaround

            // Start the Dear ImGui frame
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
            {
                static float f = 0.0f;
                static int counter = 0;

                ImGui::Begin("Mass Spring Damper System");                          // Create a window called "Hello, world!" and append into it.

                ImGui::Text("System Parameters");
                ImGui::SliderFloat("Mass", &M, 0.01f, 10.0f);
                ImGui::SliderFloat("Spring Constant", &K, 0.01f, 10.0f);
                ImGui::SliderFloat("Damping Coefficient", &b, 0.0f, 10.0f);
                ImGui::SliderFloat("Neutral Length", &L, -5.0f, 5.0f);

                //ImGui::Checkbox("Damping", &show_demo_window);      // Edit bools storing our window open/close state

                if (ImGui::Button("Reset")) {                          // Buttons return true when clicked (most widgets return true when edited/activated)
                    counter++;
                    x.x = 1.5f;
                    x.y = 0.0f;
                }
                ImGui::SameLine();
                ImGui::Text("counter = %d", counter);

                /* PlotLines(
                    const char* label, const float* values, int values_count, int values_offset = 0, 
                    const char* overlay_text = NULL, float scale_min = FLT_MAX, float scale_max = FLT_MAX, 
                    ImVec2 graph_size = ImVec2(0, 0), 
                    int stride = sizeof(float));
                */
                ImGui::PlotLines(" ", logger_x1.samples, (int)logger_x1.m_size, logger_x1.loc, "Position", -1.5f, 1.5f, ImVec2(200, 50));
                ImGui::PlotLines(" ", logger_x2.samples, (int)logger_x2.m_size, logger_x2.loc, "Velocity", -1.5f, 1.5f, ImVec2(200, 50));

                ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
                ImGui::End();
            }

            // Rendering
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            window->SwapBuffers();
            m_clock.Advance();
            window->ProcessEvents();
        }

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