# Adaptive rendering via a smooth hierarchical atlas and virtualized geometry images

Code is split into three binaries: common library, converter executable and renderer executable. Converter currently supports _some_ obj models and converts it into and atlas of geometry images. Renderer builds and displays a hierarchical atlas of geometry images.

## Building

Building is done via CMake. Confirmed to work in both VS2019 on Windows and Clion on Ubuntu, provided that libraries are configured correctly. Most libraries are provided via git submodules in "thirdparty" folder, but Eigen3, GLFW and Vulkan are not. Both of them  can be  installed via a package manager on most Linux distros.
```bash
sudo apt install libeigen3-dev libvulkan-dev
```
On  Windows, manually download these libraries and specify their install paths via CMake for windows or VS' built-in cmake build config json.
