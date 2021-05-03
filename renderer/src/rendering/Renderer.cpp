#include "Renderer.hpp"


#include <iostream>
#include <unordered_set>


constexpr std::array DEVICE_EXTENSIONS{VK_KHR_SWAPCHAIN_EXTENSION_NAME};


QueueFamilyIndices chose_queue_families(const vk::PhysicalDevice& device, const vk::SurfaceKHR& surface)
{
    auto queues = device.getQueueFamilyProperties();

    constexpr uint32_t INVALID = std::numeric_limits<uint32_t>::max();
    QueueFamilyIndices result{INVALID, INVALID};

    uint32_t queue_idx = 0;
    for (const auto& queue : queues)
    {
        if (queue.queueFlags & vk::QueueFlagBits::eGraphics)
        {
            result.graphics_queue_idx = queue_idx;
        }

        if (device.getSurfaceSupportKHR(queue_idx , surface))
        {
            result.presentation_queue_idx = queue_idx;
        }

        ++queue_idx;
    }

    if (result.graphics_queue_idx == INVALID
        || result.presentation_queue_idx == INVALID)
    {
        throw std::runtime_error("Chosen physical device does not support a graphics queue!");
    }

    return result;
}

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

    auto props = physical_device.getProperties();
    std::cout << "Max bound desc sets: " << props.limits.maxBoundDescriptorSets << std::endl;

    queue_family_indices = chose_queue_families(physical_device, display_surface.get());    

    // kostyl due to bad API design
    float queue_priority = 1.0f;
    std::vector<vk::DeviceQueueCreateInfo> queue_create_infos;
    {
        std::unordered_set<uint32_t> unique_families{
            queue_family_indices.graphics_queue_idx,
            queue_family_indices.presentation_queue_idx
        };

        queue_create_infos.reserve(unique_families.size());
        for (auto idx : unique_families)
        {
            queue_create_infos.emplace_back(vk::DeviceQueueCreateInfo{{}, idx, 1, &queue_priority});
        }
    }


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
    
    graphics_queue = device->getQueue(queue_family_indices.graphics_queue_idx, 0);
    present_queue = device->getQueue(queue_family_indices.presentation_queue_idx, 0);
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

void Renderer::create_swapchain()
{
    auto surface_caps = physical_device.getSurfaceCapabilitiesKHR(display_surface.get());
    auto format = chose_surface_foramat(physical_device, display_surface.get());
    auto present_mode = chose_present_mode(physical_device, display_surface.get());
    auto extent = chose_swap_extent(surface_caps);

    uint32_t image_count = surface_caps.minImageCount + 1;

    if (surface_caps.maxImageCount > 0)
    {
        image_count = std::min(image_count, surface_caps.maxImageCount);
    }

    bool queues_differ =
        queue_family_indices.graphics_queue_idx != queue_family_indices.presentation_queue_idx;

    std::vector<uint32_t> queue_families;
    if (queues_differ)
    {
        queue_families = {queue_family_indices.graphics_queue_idx, queue_family_indices.presentation_queue_idx};
    }

    swapchain = device->createSwapchainKHRUnique(vk::SwapchainCreateInfoKHR{
        {}, display_surface.get(),
        image_count, format.format, format.colorSpace,
        extent, 1, vk::ImageUsageFlagBits::eColorAttachment,
        queues_differ ? vk::SharingMode::eConcurrent : vk::SharingMode::eExclusive,
        static_cast<uint32_t>(queue_families.size()), queue_families.data(),
        surface_caps.currentTransform, vk::CompositeAlphaFlagBitsKHR::eOpaque,
        present_mode,
        /* clipped */ true,
        /* old swapchain */ nullptr
    });

    swapchain_images = device->getSwapchainImagesKHR(swapchain.get());
    swapchain_format = format.format;
    swapchain_extent = extent;
    
    swapchain_image_views.clear();
    swapchain_image_views.reserve(swapchain_images.size());
    for (auto& image : swapchain_images)
    {
        swapchain_image_views.emplace_back(device->createImageViewUnique(vk::ImageViewCreateInfo{
            {}, image, vk::ImageViewType::e2D, format.format,
            vk::ComponentMapping{},
            vk::ImageSubresourceRange{vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1}
        }));
    }
}
