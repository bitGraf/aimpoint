#include "Application.h"

#include "KeyCodes.h"

#include "Math/RK4.h"

#define BIND_EVENT_FN(fn) [this](auto&&... args) -> decltype(auto) { return this->fn(std::forward<decltype(args)>(args)...); }

glm::vec2 mass_spring_damper(float t, glm::vec2 x) {
    // system parameters
    float b = 0.0f;
    float M = 1.0f;
    float K = 1.0f;
    float L = 1.0f;

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

        m_clock.Create(0.005, 0.0);

        aimpoint::Window* window = aimpoint::Window::Create();
        window->SetEventCallback(BIND_EVENT_FN(Application::HandleEvent));

        // Initialize mass-spring-damper system
        glm::vec2 x(1.5, 0);

        while (!m_done) {
            LOG_TRACE("t={:08.3f}x={:18.4f}v={:28.4f}", m_clock.GetTime(), x.x, x.y);
            x = Integrate_RK4(mass_spring_damper, x, m_clock);

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