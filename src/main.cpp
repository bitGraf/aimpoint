#include "spdlog/spdlog.h"

int main()
{
    spdlog::info("Welcome to spdlog!");
    spdlog::error("Some error message with arg: {}", 1);

    spdlog::warn("Easy padding in numbers like {:08d}", 12);
    spdlog::critical("Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
    spdlog::info("Support for floats {:03.2f}", 1.23456);
    spdlog::info("Positional args are {1} {0}..", "too", "supported");
    spdlog::info("{:<30}", "left aligned");

    spdlog::set_level(spdlog::level::debug); // Set global log level to debug
    spdlog::debug("This message should be displayed..");

    // change log pattern
    spdlog::set_pattern("[%H:%M:%S %z] [%n] [%^---%L---%$] [thread %t] %v");

    // Compile time log levels
    // define SPDLOG_ACTIVE_LEVEL to desired level
    SPDLOG_TRACE("Some trace message with param {}", 42);
    SPDLOG_DEBUG("Some debug message");

    system("pause");
}

/*
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
*/