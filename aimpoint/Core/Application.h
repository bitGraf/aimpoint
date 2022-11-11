#include "Window.h"
#include "EventTypes.h"
#include "Timing/Clock.h"

#include "solver/RigidBodySystem.h"

namespace aimpoint {

	class Application {
	public:
		Application();
		~Application();

		void Run();
		void Close();

	private:
		void HandleEvent(aimpoint::Event& event);
		
		bool OnKeyPressedEvent(KeyPressedEvent& event);
		bool OnWindowClose(WindowCloseEvent& event);
		bool OnWindowResize(WindowResizeEvent& event);

	private:
		bool m_done;
		bool m_minimized;

		Window* m_window;
		Clock m_clock;

		ab_solver::RigidBodySystem m_system;
		ab_solver::OdeSolver m_solver;
		std::vector<ab_solver::RigidBody*> m_bodies;
	};
}