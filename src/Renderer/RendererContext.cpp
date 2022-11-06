#include "RendererContext.h"

#include "Platform/OpenGL/OpenGLContext.h"

namespace aimpoint {

	RendererContext* RendererContext::Create(GLFWwindow* windowHandle) {
		/*switch (RendererAPI::Current())
		{
			case RendererAPIType::None:    return nullptr;
			case RendererAPIType::OpenGL:  return Ref<OpenGLContext>::Create(windowHandle);
			case RendererAPIType::Vulkan:  return Ref<VulkanContext>::Create(windowHandle);
		}
		HZ_CORE_ASSERT(false, "Unknown RendererAPI");
		return nullptr;
        */
       //return OpenGLContext::Create(windowHandle);
		OpenGLContext* context = new OpenGLContext(windowHandle);
		context->Create();
		return context;
	}

}