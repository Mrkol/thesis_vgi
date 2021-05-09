#include "Application.hpp"

#include <backends/imgui_impl_glfw.h>
#include <cxxopts.hpp>

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
    parse_arguments(argc, argv);

    glfwSetWindowUserPointer(main_window.get(), this);
    glfwSetFramebufferSizeCallback(main_window.get(), on_window_resized);
    glfwSetKeyCallback(main_window.get(), on_key_event);
    glfwSetMouseButtonCallback(main_window.get(), on_mouse_button);

    VkHelpers::check_validation_layers_support(VALIDATION_LAYERS);

    vk::ApplicationInfo application_info{
        APP_NAME, 1u,
        "Vulkan.hpp", 1u,
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

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForVulkan(main_window.get(), true);

    renderer = std::make_unique<Renderer>(vulkan_instance.get(),
        vk::UniqueSurfaceKHR{surface,
            vk::ObjectDestroy<vk::Instance, vk::DispatchLoaderStatic>{vulkan_instance.get()}},
        std::span{VALIDATION_LAYERS}.subspan<0>(),
        [this]()
        {
            int width, height;
            glfwGetFramebufferSize(main_window.get(), &width, &height);

            while (width == 0 || height == 0)
            {
                glfwGetFramebufferSize(main_window.get(), &width, &height);
                glfwWaitEvents();
            }

            return vk::Extent2D{static_cast<uint32_t>(width), static_cast<uint32_t>(height)};
        });
}

int Application::run()
{
    // kostyl
    glfwPollEvents();
    prev_mouse_pos = poll_cursor();
    while(!glfwWindowShouldClose(main_window.get()))
    {
        glfwPollEvents();


        ImGui_ImplGlfw_NewFrame();
        tick();

        renderer->render();
    }

    return 0;
}

void Application::tick()
{
    auto* cam = renderer->debug_get_scene()->debug_get_camera();
    cam->move(cam_velocity.cast<float>());

    {
        auto c = poll_cursor();

        if (move_camera)
        {
            Eigen::Vector2f d = c - prev_mouse_pos;
            cam->rotate(d.x(), d.y());
        }

        prev_mouse_pos = c;
    }

}

Application::Config Application::parse_arguments(int argc, char** argv)
{
    cxxopts::Options options("VGI renderer", "");
    
    options.add_options()
        ("d,diagnostics", "Enable diagnostic printing", cxxopts::value<bool>())
    ;

    auto parsed = options.parse(argc, argv);

    return {
        parsed["d"].as<bool>()
    };
}

void Application::on_window_resized(GLFWwindow* window, int width, int height)
{
    auto app = reinterpret_cast<Application*>(glfwGetWindowUserPointer(window));
    app->renderer->on_window_resized();
}

void Application::on_key_event(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (ImGui::GetIO().WantCaptureKeyboard)
    {
        return;
    }
    auto app = reinterpret_cast<Application*>(glfwGetWindowUserPointer(window));

    if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
        return;
    }

    int coeff = action == GLFW_PRESS ? 1 : (action == GLFW_RELEASE ? -1 : 0);
    app->cam_velocity.y() += coeff*(key == GLFW_KEY_W)     - coeff*(key == GLFW_KEY_S);
    app->cam_velocity.x() += coeff*(key == GLFW_KEY_A)     - coeff*(key == GLFW_KEY_D);
    app->cam_velocity.z() += coeff*(key == GLFW_KEY_SPACE) - coeff*(key == GLFW_KEY_LEFT_SHIFT);
}

Eigen::Vector2f Application::poll_cursor()
{
    double x, y;
    glfwGetCursorPos(main_window.get(), &x, &y);
    return {x, y};
}

void Application::on_mouse_button(GLFWwindow* window, int button, int action, int mods)
{
    if (ImGui::GetIO().WantCaptureMouse)
    {
        return;
    }

    auto app = reinterpret_cast<Application*>(glfwGetWindowUserPointer(window));

    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (action == GLFW_PRESS)
        {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            app->move_camera = true;
        }
        else if (action == GLFW_RELEASE)
        {
            app->move_camera = false;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        }
    }
}
