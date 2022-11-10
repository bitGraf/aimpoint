#pragma once

#include "Event.h"
#include "Renderer/RendererContext.h"

namespace aimpoint {

	struct WindowProps
	{
		std::string Title;
		unsigned int Width;
		unsigned int Height;

		WindowProps(const std::string& title = "Window",
			unsigned int width = 1280,
			unsigned int height = 720)
			: Title(title), Width(width), Height(height) {
		}
	};

	class Window {
	public:
        using EventCallbackFcn = std::function<void(Event&)>;
		
		virtual ~Window() {}

        virtual void ProcessEvents() = 0;
		virtual void SwapBuffers() = 0;

        /* Access Window parameters */
        virtual uint32_t GetWidth() const = 0;
        virtual uint32_t GetHeight() const = 0;
		virtual std::pair<uint32_t, uint32_t> GetSize() const = 0;
		virtual std::pair<float, float> GetWindowPos() const = 0;

        virtual void SetEventCallback(const EventCallbackFcn& callback) = 0;
        virtual void SetVSync(bool enabled) = 0;
        virtual bool IsVSync() const = 0;

		virtual const std::string& GetTitle() const = 0;
		virtual void SetTitle(const std::string& title) = 0;

        virtual void* GetNativeWindow() const = 0;

        virtual RendererContext* GetRenderContext() = 0;

        /* Create a new window */
        static Window* Create(const WindowProps& props = WindowProps());
	};

}