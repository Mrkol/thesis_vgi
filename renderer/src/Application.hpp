#pragma once


#include <vulkan/vulkan.hpp>
#include <GLFW/glfw3.h>
#include <memory>
#include <chrono>

#include <Utility.hpp>
#include "rendering/Renderer.hpp"


class ApplicationBase {
public:
    ApplicationBase()
    {
        if (glfwInit() != GLFW_TRUE) {
            AD_HOC_PANIC("Unable to initialize GLFW!");
        }
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    }

    ~ApplicationBase()
    {
        glfwTerminate();
    }

    ApplicationBase(const ApplicationBase&) = delete;
    ApplicationBase(ApplicationBase&&) = delete;
    ApplicationBase& operator=(const ApplicationBase&) = delete;
    ApplicationBase& operator=(ApplicationBase&&) = delete;
};

class Application : private ApplicationBase {
    struct Config
    {
        bool print_diagnostics;
    };

public:
    Application(int argc, char** argv);

    int run();

private:
    void tick(float delta_seconds);

    Eigen::Vector2f poll_cursor();

    Config parse_arguments(int argc, char** argv);
    static void on_window_resized(GLFWwindow* window, int width, int height);
    static void on_key_event(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void on_mouse_button(GLFWwindow* window, int button, int action, int mods);

private:
    using Clock = std::chrono::steady_clock;
    Clock::time_point last_tick;

    std::unique_ptr<GLFWwindow, void (*)(GLFWwindow*)> main_window;
    vk::UniqueInstance vulkan_instance;
    std::unique_ptr<Renderer> renderer;

    Eigen::Vector3i cam_velocity{0, 0, 0};
    Eigen::Vector2f prev_mouse_pos{0, 0};
    bool move_camera{false};

    bool shader_hotswap_requested{false};
};
