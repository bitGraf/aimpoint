#include "window.h"

#define BIND_EVENT_FN(fn) [this](auto&&... args) -> decltype(auto) { return this->fn(std::forward<decltype(args)>(args)...); }

/*
void HandleEvent(Event& event) {
    EventDispatcher dispatcher(event);
    dispatcher.Dispatch<WindowCloseEvent>(BIND_EVENT_FN(Application::OnWindowClose));
    dispatcher.Dispatch<WindowResizeEvent>(BIND_EVENT_FN(Application::OnWindowResize));

    for (auto it = m_layerStack.rbegin(); it != m_layerStack.rend(); ++it) {
        if ((*it)->IsActive()) {
            (*it)->OnEvent(event);
            if (event.Handled)
                break;
        }
    }
}
*/

int main(void)
{
    aimpoint::Window* window = aimpoint::Window::Create("Title", 600, 800);
    //window->SetEventCallback(BIND_EVENT_FN(HandleEvent));

    bool done = false;
    while (!window->ShouldClose()) {
        window->Update();
    }

    return 0;
}