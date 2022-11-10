#include "Window.h"
#include "EventTypes.h"
#include "Timing/Clock.h"

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
	};
}