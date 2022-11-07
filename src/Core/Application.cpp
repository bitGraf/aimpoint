#include "Application.h"

#include "KeyCodes.h"

#include "Math/RK4.h"
#include "Math/SignalLogger.h"

#include <glfw/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "implot.h"

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

aimpoint::State6DoF rigid_body(float t, aimpoint::State6DoF x) {
    glm::vec3 net_force(0, -9.81, 0);
    float mass = 1;
    glm::vec3 net_moment(0, 0, 0);
    glm::vec3 inertia_p(.2f, .3f, .4f);

    aimpoint::State6DoF x_dot;

    // Newtons law
    x_dot.Position = x.Velocity;
    x_dot.Velocity = net_force / mass;

    // Euler's equations of motion
    x_dot.BodyRate.x = (net_moment.x - (inertia_p.z-inertia_p.y)*x.BodyRate.y*x.BodyRate.z) / inertia_p.x;
    x_dot.BodyRate.y = (net_moment.y - (inertia_p.x-inertia_p.z)*x.BodyRate.z*x.BodyRate.x) / inertia_p.y;
    x_dot.BodyRate.z = (net_moment.z - (inertia_p.y-inertia_p.x)*x.BodyRate.x*x.BodyRate.y) / inertia_p.z;

    // Get quaterion-rate from body rate
    float ep = 1 - x.Orientation.x*x.Orientation.x - x.Orientation.y*x.Orientation.y - x.Orientation.z*x.Orientation.z - x.Orientation.w*x.Orientation.w;
    float K = 1;

    x_dot.Orientation.w = -x.BodyRate.x*x.Orientation.x - x.BodyRate.y*x.Orientation.y - x.BodyRate.z*x.Orientation.z + K*ep*x.Orientation.w;
    x_dot.Orientation.x =  x.BodyRate.x*x.Orientation.w - x.BodyRate.z*x.Orientation.y - x.BodyRate.y*x.Orientation.z + K*ep*x.Orientation.x;
    x_dot.Orientation.y =  x.BodyRate.y*x.Orientation.w - x.BodyRate.z*x.Orientation.x - x.BodyRate.x*x.Orientation.z + K*ep*x.Orientation.y;
    x_dot.Orientation.z =  x.BodyRate.z*x.Orientation.w - x.BodyRate.y*x.Orientation.x - x.BodyRate.x*x.Orientation.y + K*ep*x.Orientation.z;

    //qd1 = -w1*q2 - w2*q3 - w3*q4 + K*ep*q1;
    //qd2 =  w1*q1 - w3*q3 - w2*q4 + K*ep*q2;
    //qd3 =  w2*q1 - w3*q2 - w1*q4 + K*ep*q3;
    //qd4 =  w3*q1 - w2*q2 - w1*q3 + K*ep*q4;

    //x_dot.Orientation.w = 0.5f * (           0*x.Orientation.w - x.BodyRate.x*x.Orientation.w - x.BodyRate.y*x.Orientation.w - x.BodyRate.z*x.Orientation.w);
    //x_dot.Orientation.x = 0.5f * (x.BodyRate.x*x.Orientation.x +            0*x.Orientation.x + x.BodyRate.z*x.Orientation.x - x.BodyRate.y*x.Orientation.x);
    //x_dot.Orientation.y = 0.5f * (x.BodyRate.y*x.Orientation.y - x.BodyRate.z*x.Orientation.y +            0*x.Orientation.y + x.BodyRate.x*x.Orientation.y);
    //x_dot.Orientation.z = 0.5f * (x.BodyRate.z*x.Orientation.z + x.BodyRate.y*x.Orientation.z - x.BodyRate.x*x.Orientation.z +            0*x.Orientation.z);

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

        // Simulations params

        m_clock.Create(1/120.0, 0.0);

        aimpoint::Window* window = aimpoint::Window::Create();
        window->SetEventCallback(BIND_EVENT_FN(Application::HandleEvent));

        // Initialize mass-spring-damper system
        State6DoF x;
        x.Position = glm::vec3(0, 0, 0);
        x.Velocity = glm::vec3(1, 1, 0);
        x.Orientation = glm::vec4(0, 0, 0, 1);
        x.BodyRate = glm::vec3(0, 0, 0);

        SignalLogger<250> logger_x;
        SignalLogger<250> logger_y;

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

            // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
            {
                static float f = 0.0f;
                static int counter = 0;

                ImGui::Begin("Mass Spring Damper System");                          // Create a window called "Hello, world!" and append into it.

                //ImGui::Text("System Parameters");
                //ImGui::SliderFloat("Mass", &M, 0.01f, 10.0f);
                //ImGui::SliderFloat("Spring Constant", &K, 0.01f, 10.0f);
                //ImGui::SliderFloat("Damping Coefficient", &b, 0.0f, 10.0f);
                //ImGui::SliderFloat("Neutral Length", &L, -5.0f, 5.0f);
                ////ImGui::Text("Period: %6.3f sec", 2.0f*3.14159f*sqrt(M/K));
                //ImGui::Text("Period: %.6f sec", sqrt(x.Orientation.x*x.Orientation.x + x.Orientation.w*x.Orientation.w + x.Orientation.y*x.Orientation.y + x.Orientation.z*x.Orientation.z));

                //ImGui::Checkbox("Damping", &show_demo_window);      // Edit bools storing our window open/close state

                if (ImGui::Button("Reset")) {                          // Buttons return true when clicked (most widgets return true when edited/activated)
                    x.Position = glm::vec3(0, 0, 0);
                    x.Velocity = glm::vec3(30, 30, 0);
                    x.Orientation = glm::vec4(0, 0, 0, 1);
                    x.BodyRate = glm::vec3(0, 0, 0);
                }
                //ImGui::SameLine();
                //ImGui::Text("counter = %d", counter);

                logger_x.AddSample(x.Position.x);
                logger_y.AddSample(x.Position.y);

                if (ImPlot::BeginPlot("My Plot")) {
                    ImPlot::PlotLine("Trajectory", logger_x.samples, logger_y.samples, logger_x.m_size, 0, logger_x.loc);
                    ImPlot::PlotScatter("Trajectory", logger_x.samples, logger_y.samples, logger_x.m_size, 0, logger_x.loc);
                    ImPlot::EndPlot();
                }

                //ImPlot::ShowDemoWindow();
                //ImGui::Text("Quaternion");
                //ImGui::PlotLines(" ", logger_q1.samples, (int)logger_q1.m_size, logger_q1.loc, "Orientation:1", -1.0f, 1.0f, ImVec2(200, 50));
                //ImGui::PlotLines(" ", logger_q2.samples, (int)logger_q2.m_size, logger_q2.loc, "Orientation:2", -1.0f, 1.0f, ImVec2(200, 50));
                //ImGui::PlotLines(" ", logger_q3.samples, (int)logger_q3.m_size, logger_q3.loc, "Orientation:3", -1.0f, 1.0f, ImVec2(200, 50));
                //ImGui::PlotLines(" ", logger_q4.samples, (int)logger_q4.m_size, logger_q4.loc, "Orientation:4", -1.0f, 1.0f, ImVec2(200, 50));
                //ImGui::Text("Euler");
                //ImGui::PlotLines(" ", logger_yaw.samples, (int)logger_yaw.m_size, logger_yaw.loc, "Yaw", -180.0f, 180.0f, ImVec2(200, 50));
                //ImGui::PlotLines(" ", logger_pitch.samples, (int)logger_pitch.m_size, logger_pitch.loc, "Pitch", -180.0f, 180.0f, ImVec2(200, 50));
                //ImGui::PlotLines(" ", logger_roll.samples, (int)logger_roll.m_size, logger_roll.loc, "Roll", -180.0f, 180.0f, ImVec2(200, 50));

                ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
                ImGui::End();
            }

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
            x = Integrate_RK4(rigid_body, x, m_clock);
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