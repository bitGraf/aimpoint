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

        // Create simulation vars
        m_clock.Create(1/120.0, 0.0);
        m_system.Initialize(&m_solver);

        ab_solver::RigidBody* b1 = new ab_solver::RigidBody(); 
        b1->pos_x = -5.0;
        b1->vel_x = 5.0;
        b1->vel_y = 5.0;
        m_bodies.push_back(b1); m_system.AddRigidBody(b1);
        ab_solver::RigidBody* b2 = new ab_solver::RigidBody(); 

        m_bodies.push_back(b2); m_system.AddRigidBody(b2);
        ab_solver::RigidBody* b3 = new ab_solver::RigidBody(); 

        m_bodies.push_back(b3); m_system.AddRigidBody(b3);

        // Create window and bind event callback fcn
        aimpoint::Window* window = aimpoint::Window::Create();
        window->SetEventCallback(BIND_EVENT_FN(Application::HandleEvent));

        // TODO: get out of here
        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImPlot::CreateContext();

        ImGuiIO& io = ImGui::GetIO(); (void)io;
        //io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        ImGui::StyleColorsDark();
        ImPlot::StyleColorsDark();

        // Setup Platform/Renderer backends
        ImGui_ImplGlfw_InitForOpenGL((GLFWwindow*)window->GetNativeWindow(), true);
        ImGui_ImplOpenGL3_Init("#version 460");

        while (!m_done) {
            // Do simulation
            m_system.Process(m_clock.GetTimestep(), 1);

            // Do rendering
            window->GetRenderContext()->BeginFrame(); //TODO: temp workaround

            // Start the Dear ImGui frame
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            // Print out rigid body states
            if (ImGui::Begin("RigidBodySystem")) {
                ImGui::Text("dT = %5.3fs [%6.2f Hz], T = %7.3fs", m_clock.GetTimestep(), 1.0/ m_clock.GetTimestep(), m_clock.GetTime());
                ImGui::Separator();
                for (int n = 0; n < m_system.GetRigidBodyCount(); n++) {
                    auto body = m_system.GetRigidBody(n);
                    ImGui::Text("Body: %02d", n);
                    //ImGui::SameLine();
                    ImGui::Text("    mass: %5.2f", body->mass);
                    //ImGui::SameLine();
                    ImGui::Text("    pos: <%5.2f,%5.2f,%5.2f>", body->pos_x, body->pos_y, body->pos_z);
                    //ImGui::SameLine();
                    ImGui::Text("    vel: <%5.2f,%5.2f,%5.2f>", body->vel_x, body->vel_y, body->vel_z);
                    ImGui::Separator();
                }
            }
            ImGui::End();

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

        for (auto* body : m_bodies) {
            delete body;
        }
        m_bodies.clear();

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