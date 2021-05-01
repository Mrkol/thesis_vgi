#include "Renderer.hpp"


constexpr std::array DEVICE_EXTENSIONS{VK_KHR_SWAPCHAIN_EXTENSION_NAME};


vk::SurfaceFormatKHR chose_surface_foramat(const vk::PhysicalDevice& device, const vk::SurfaceKHR& surface)
{
    auto formats = device.getSurfaceFormatsKHR(surface);

    if (formats.empty())
    {
        throw std::runtime_error("Device does not support any surface formats!");
    }

    auto found = std::find_if(formats.begin(), formats.end(),
        [](const vk::SurfaceFormatKHR& format)
        {
            return format.format == vk::Format::eB8G8R8A8Srgb
                && format.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear;
        });

    return found != formats.end() ? *found : formats[0];
}

vk::PresentModeKHR chose_present_mode(const vk::PhysicalDevice& device, const vk::SurfaceKHR& surface)
{
    auto modes = device.getSurfacePresentModesKHR(surface);

    if (modes.empty())
    {
        throw std::runtime_error("Device doesn't support any present modes!");
    }

    return std::find(modes.begin(), modes.end(), vk::PresentModeKHR::eMailbox) != modes.end() ?
        vk::PresentModeKHR::eMailbox : vk::PresentModeKHR::eFifo;
}

Renderer::Renderer(vk::Instance& instance, vk::UniqueSurfaceKHR surface,
    std::span<const char* const> validation_layers,
    fu2::unique_function<vk::Extent2D()> res_provider)
    : resolution_provider{std::move(res_provider)}
    , vulkan_instance{&instance}
    , display_surface{std::move(surface)}
{
    // TODO: Proper device selection
    physical_device = vulkan_instance->enumeratePhysicalDevices().front();


    auto queues = physical_device.getQueueFamilyProperties();

    uint32_t queue_idx = 0;
    for (const auto& queue : queues)
    {
        // TODO: Maybe separate graphics and presentation queues?
        if (queue.queueFlags & vk::QueueFlagBits::eGraphics
            && physical_device.getSurfaceSupportKHR(queue_idx, display_surface.get()))
        {
            break;
        }
        ++queue_idx;
    }

    if (queue_idx == queues.size())
    {
        throw std::runtime_error("No suitable logic device found!");
    }

    std::array queue_create_infos {
        vk::DeviceQueueCreateInfo{
            {}, queue_idx, 1
        }
    };

    vk::PhysicalDeviceFeatures features {
    };

    // NOTE: newer vulkan implementations ignore the layers here
    device = physical_device.createDeviceUnique(vk::DeviceCreateInfo{
        {},
        static_cast<uint32_t>(queue_create_infos.size()), queue_create_infos.data(),
        static_cast<uint32_t>(validation_layers.size()), validation_layers.data(),
        static_cast<uint32_t>(DEVICE_EXTENSIONS.size()), DEVICE_EXTENSIONS.data(),
        &features
    });

    graphics_queue = device->getQueue(queue_idx, 0);
}

void Renderer::render()
{

}

vk::Extent2D Renderer::chose_swap_extent(const vk::SurfaceCapabilitiesKHR& capabilities)
{
    if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
        return capabilities.currentExtent;
    }

    VkExtent2D actualExtent = resolution_provider();

    actualExtent.width = std::max(capabilities.minImageExtent.width, std::min(capabilities.maxImageExtent.width, actualExtent.width));
    actualExtent.height = std::max(capabilities.minImageExtent.height, std::min(capabilities.maxImageExtent.height, actualExtent.height));

    return actualExtent;
}
