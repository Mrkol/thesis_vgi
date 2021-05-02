#pragma once


#include <vulkan/vulkan.hpp>
#include <GLFW/glfw3.h>
#include <memory>
#include <Utility.hpp>

#include "Renderer.hpp"


class ApplicationBase {
public:
    ApplicationBase()
    {
        if (glfwInit() != GLFW_TRUE) {
            AD_HOC_PANIC("Unable to initialize GLFW!");
        }
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
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
public:
    Application(int argc, char** argv);

    int run();

private:
    std::unique_ptr<GLFWwindow, void (*)(GLFWwindow*)> main_window;
    vk::UniqueInstance vulkan_instance;
    std::unique_ptr<Renderer> renderer;
};
