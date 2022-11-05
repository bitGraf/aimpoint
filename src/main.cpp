#include "Window.h"
#include "EventTypes.h"

//#define BIND_EVENT_FN(fn) [this](auto&&... args) -> decltype(auto) { return this->fn(std::forward<decltype(args)>(args)...); }
#define BIND_EVENT_FN(fn) [](auto&&... args) -> decltype(auto) { return fn(std::forward<decltype(args)>(args)...); }

void HandleEvent(aimpoint::Event& event) {
    aimpoint::EventDispatcher dispatcher(event);
    //dispatcher.Dispatch<aimpoint::WindowCloseEvent>(BIND_EVENT_FN(Application::OnWindowClose));
    //dispatcher.Dispatch<aimpoint::WindowResizeEvent>(BIND_EVENT_FN(Application::OnWindowResize));
}

int main(void)
{
    aimpoint::Window* window = aimpoint::Window::Create("Title", 600, 800);
    window->SetEventCallback(BIND_EVENT_FN(HandleEvent));

    bool done = false;
    while (!window->ShouldClose()) {
        window->Update();
    }

    return 0;
}