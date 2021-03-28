#pragma once

#include <thread>

#include "function2/function2.hpp"

#include "UnboundedBlockingMPMCQueue.hpp"
#include "ResourceCounter.hpp"


class StaticThreadPool
{
public:
    using Job = fu2::unique_function<void()>;

    explicit StaticThreadPool(std::size_t size = std::thread::hardware_concurrency());

    void submit(Job job);

    static constexpr std::size_t THREAD_NONE = std::numeric_limits<std::size_t>::max();
    static std::size_t current_thread_index();

    ~StaticThreadPool();

private:
    std::vector<std::thread> workers;
    UnboundedBlockingMPMCQueue<Job> jobs;
    ResourceCounter outstanding_work;
};

