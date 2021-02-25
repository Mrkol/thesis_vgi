#pragma once

#include <chrono>
#include <iostream>
#include <string_view>


class ScopedTimer
{
    using Clock = std::chrono::high_resolution_clock;
public:
    explicit ScopedTimer(std::string_view name)
        : name(name)
    {
        std::cout << "Starting " << name << "...\n";
        started = Clock::now();
    }

    ~ScopedTimer()
    {
        auto ended = Clock::now();
        std::cout << "Finished " << name << " in "
            << std::chrono::duration_cast<std::chrono::milliseconds>(ended - started).count()
            << "ms.\n";
    }
private:
    std::string_view name;
    std::chrono::time_point<Clock> started;
};
