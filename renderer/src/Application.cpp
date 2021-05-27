#include "Application.hpp"

#include <backends/imgui_impl_glfw.h>
#include <cxxopts.hpp>

#include <VkHelpers.hpp>


#ifdef NDEBUG
constexpr std::array VALIDATION_LAYERS {"VK_LAYER_KHRONOS_validation"};
#else
constexpr std::array VALIDATION_LAYERS {"VK_LAYER_KHRONOS_validation"};
#endif

constexpr std::array EXTENSIONS {VK_KHR_SWAPCHAIN_EXTENSION_NAME, "VK_EXT_debug_utils"};

constexpr const char* APP_NAME = "VGI renderer";




Application::Application(int argc, char** argv)
    : last_tick_(Clock::now())
    , main_window_{glfwCreateWindow(800, 600, APP_NAME, nullptr, nullptr), &glfwDestroyWindow}
{
    parse_arguments(argc, argv);

    glfwSetWindowUserPointer(main_window_.get(), this);
    glfwSetFramebufferSizeCallback(main_window_.get(), on_window_resized);
    glfwSetKeyCallback(main_window_.get(), on_key_event);
    glfwSetMouseButtonCallback(main_window_.get(), on_mouse_button);

    VkHelpers::check_validation_layers_support(VALIDATION_LAYERS);

    vk::ApplicationInfo application_info{
        APP_NAME, 1u,
        "Vulkan.hpp", 1u,
        VK_API_VERSION_1_2
    };

    uint32_t glfwExtCount;
    auto glfwExts = glfwGetRequiredInstanceExtensions(&glfwExtCount);

    vulkan_instance_ = vk::createInstanceUnique(vk::InstanceCreateInfo{
        {}, &application_info,
        static_cast<uint32_t>(VALIDATION_LAYERS.size()), VALIDATION_LAYERS.data(),
        glfwExtCount, glfwExts
    });

    VkSurfaceKHR surface;
    if (glfwCreateWindowSurface(VkInstance(vulkan_instance_.get()), main_window_.get(), nullptr, &surface) != VK_SUCCESS) {
        AD_HOC_PANIC("Unable to create VK surface!");
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForVulkan(main_window_.get(), true);

    renderer_ = std::make_unique<Renderer>(vulkan_instance_.get(),
        vk::UniqueSurfaceKHR{surface,
            vk::ObjectDestroy<vk::Instance, vk::DispatchLoaderStatic>{vulkan_instance_.get()}},
        std::span{VALIDATION_LAYERS}.subspan<0>(),
        [this]()
        {
            int width, height;
            glfwGetFramebufferSize(main_window_.get(), &width, &height);

            while (width == 0 || height == 0)
            {
                glfwGetFramebufferSize(main_window_.get(), &width, &height);
                glfwWaitEvents();
            }

            return vk::Extent2D{static_cast<uint32_t>(width), static_cast<uint32_t>(height)};
        });
}

int Application::run()
{
    // kostyl
    glfwPollEvents();
    prev_mouse_pos_ = poll_cursor();
    while(!glfwWindowShouldClose(main_window_.get()))
    {
        glfwPollEvents();


        ImGui_ImplGlfw_NewFrame();
        auto this_tick = Clock::now();
        float delta_seconds =
            std::chrono::duration_cast<std::chrono::duration<float, std::ratio<1, 1>>>(this_tick - last_tick_).count();
        last_tick_ = this_tick;


        tick(delta_seconds);
        renderer_->render(delta_seconds);
    }

    return 0;
}

void Application::tick(float delta_seconds)
{
    auto* cam = renderer_->debug_get_scene()->debug_get_camera();
    cam->move(cam_velocity_.cast<float>(), 1.4f * delta_seconds * cam_speed_);

    {
        auto c = poll_cursor();

        if (move_camera_)
        {
            Eigen::Vector2f d = c - prev_mouse_pos_;
            cam->rotate(d.x(), d.y());
        }

        prev_mouse_pos_ = c;
    }

    if (std::exchange(shader_hotswap_requested_, false))
    {
        renderer_->hotswap_shaders();
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
    app->renderer_->on_window_resized();
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

    if (key == GLFW_KEY_R && action == GLFW_RELEASE)
    {
        app->shader_hotswap_requested_ = true;
        return;
    }

    int coeff = action == GLFW_PRESS ? 1 : (action == GLFW_RELEASE ? -1 : 0);
    app->cam_velocity_.y() += coeff*(key == GLFW_KEY_W)     - coeff*(key == GLFW_KEY_S);
    app->cam_velocity_.x() += coeff*(key == GLFW_KEY_A)     - coeff*(key == GLFW_KEY_D);
    app->cam_velocity_.z() += coeff*(key == GLFW_KEY_SPACE) - coeff*(key == GLFW_KEY_LEFT_SHIFT);
    if (key == GLFW_KEY_LEFT_CONTROL)
    {
        if (action == GLFW_PRESS)
        {
            app->cam_speed_ = 2;
        }
        else
        {
            app->cam_speed_ = 1;
        }
    }
}

Eigen::Vector2f Application::poll_cursor()
{
    double x, y;
    glfwGetCursorPos(main_window_.get(), &x, &y);
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
            app->move_camera_ = true;
        }
        else if (action == GLFW_RELEASE)
        {
            app->move_camera_ = false;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        }
    }
}
