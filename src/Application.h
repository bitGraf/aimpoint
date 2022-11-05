#include "Window.h"
#include "EventTypes.h"

namespace aimpoint {

	class Application {
	public:
		Application();
		~Application();

		void Run();

	private:
		void HandleEvent(aimpoint::Event& event);
		
		bool OnWindowClose(WindowCloseEvent& event);
		bool OnWindowResize(WindowResizeEvent& event);

	private:
		bool m_done;
		bool m_minimized;
		Window* m_window;
	};
}