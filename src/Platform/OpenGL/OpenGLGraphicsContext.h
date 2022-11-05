#pragma once

#include "GraphicsContext.h"


struct GLFWwindow;

namespace aimpoint {

    class OpenGLGraphicsContext : public GraphicsContext {
    public:
        OpenGLGraphicsContext(GLFWwindow* windowHandle);

        virtual void Init() override;
        virtual void SwapBuffers() override;

        virtual const unsigned char* GetVendorString() const override;
        virtual const unsigned char* GetDeviceString() const override;
        virtual const unsigned char* GetVersionString() const override;
        virtual const unsigned char* GetAPIVersionString() const override;

    private:
        GLFWwindow* m_WindowHandle;
    };
}