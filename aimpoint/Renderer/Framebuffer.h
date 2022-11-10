#pragma once

#include "Core/Core.h"

#include <glm/glm.hpp>

namespace aimpoint {

    enum class FramebufferFormat {
        None    = 0,
        RGBA8    = 1,
        RGBA16F = 2
    };

    struct FramebufferSpecification {
        float scale = 1.0f;
        uint32_t Width = 0;
        uint32_t Height = 0;
        glm::vec4 ClearColor = { 0.0f, 0.0f, 0.0f, 1.0f };
        FramebufferFormat Format = FramebufferFormat::RGBA8;
        uint32_t Samples = 1;

        // SwapChainTarget = screen buffer (i.e. no framebuffer)
        bool SwapChainTarget = false;

        std::string DebugName;
    };

    class Framebuffer {
    public:
        virtual ~Framebuffer() {}
        
        virtual void Bind() const = 0;
        virtual void UnBind() const = 0;

        virtual void Resize(uint32_t width, uint32_t height, bool forceRecreate = false) = 0;
        virtual void AddResizeCallback(const std::function<void(Framebuffer*)>& func) = 0;

        virtual void BindTexture(uint32_t slot = 0) const = 0;

        virtual uint32_t GetWidth() const = 0;
        virtual uint32_t GetHeight() const = 0;
        virtual uint32_t GetRendererID() const = 0;
        virtual uint32_t GetColorAttachmentRendererID() const = 0;
        virtual uint32_t GetDepthAttachmentRendererID() const = 0;

        virtual const FramebufferSpecification& GetSpecification() const = 0;

        static Framebuffer* Create(const FramebufferSpecification& spec);
    };
}