#pragma once

#include "Event.h"
#include "GraphicsContext.h"

namespace aimpoint {

	/* virtual class */
	class Window {
	public:
        using EventCallbackFcn = std::function<void(Event&)>;
		
		virtual ~Window() = default;

        /* Initialization functions*/
        virtual void Update() = 0;

        /* Access Window parameters */
        virtual unsigned int GetWidth() const = 0;
        virtual unsigned int GetHeight() const = 0;
        virtual unsigned int GetXpos() const = 0;
        virtual unsigned int GetYpos() const = 0;

        virtual void SetEventCallback(const EventCallbackFcn& callback) = 0;
        virtual void SetVSync(bool enabled) = 0;
        virtual bool IsVSync() const = 0;

        virtual bool ShouldClose() const = 0;

        virtual void* GetNativeWindow() const = 0;
        virtual GraphicsContext* GetGraphicsContext() const = 0;

        /* Create a new window */
        static Window* Create(std::string title = "Window", unsigned int height = 600, unsigned int width = 800);
	};

}