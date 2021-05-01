#include "Application.hpp"

#include <VkHelpers.hpp>


#ifdef NDEBUG
constexpr std::array<const char*, 0> VALIDATION_LAYERS {};
#else
constexpr std::array VALIDATION_LAYERS {"VK_LAYER_KHRONOS_validation"};
#endif

constexpr std::array EXTENSIONS {VK_KHR_SWAPCHAIN_EXTENSION_NAME};

constexpr const char* APP_NAME = "VGI renderer";

Application::Application(int argc, char** argv)
    : main_window{glfwCreateWindow(800, 600, APP_NAME, nullptr, nullptr), &glfwDestroyWindow}
{
    VkHelpers::check_validation_layers_support(VALIDATION_LAYERS);

    {
        vk::ApplicationInfo application_info{
            APP_NAME, 1,
            "Vulkan.hpp", 1,
            VK_API_VERSION_1_2
        };

        uint32_t glfwExtCount;
        auto glfwExts = glfwGetRequiredInstanceExtensions(&glfwExtCount);

        vulkan_instance = vk::createInstanceUnique(vk::InstanceCreateInfo{
            {}, &application_info,
            static_cast<uint32_t>(VALIDATION_LAYERS.size()), VALIDATION_LAYERS.data(),
            glfwExtCount, glfwExts
        });

        VkSurfaceKHR surface;
        if (glfwCreateWindowSurface(VkInstance(vulkan_instance.get()), main_window.get(), nullptr, &surface) != VK_SUCCESS) {
            AD_HOC_PANIC("Unable to create VK surface!");
        }

        renderer = std::make_unique<Renderer>(vulkan_instance.get(), vk::UniqueSurfaceKHR{surface},
            std::span{VALIDATION_LAYERS}.subspan<0>(),
            [this]()
            {
                int width, height;
                glfwGetFramebufferSize(main_window, width, height);
                return vk::Extent2D{static_cast<uint32_t>(width), static_cast<uint32_t>(height)};
            });
    }
}

int Application::run()
{
    while(!glfwWindowShouldClose(main_window.get()))
    {
        glfwPollEvents();

        renderer->render();
    }

    return 0;
}
