#pragma once

#include "Core.h"

struct GLFWwindow;

namespace aimpoint {

    class RendererContext {
    public:
        RendererContext() = default;
        virtual ~RendererContext() = default;

        virtual void Create() = 0;
        virtual void BeginFrame() = 0;
        virtual void SwapBuffers() = 0;

        virtual void OnResize(uint32_t width, uint32_t height) = 0;

        static RendererContext* Create(GLFWwindow* windowHandle);
    };
}