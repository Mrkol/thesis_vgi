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

    ~StaticThreadPool();

private:
    std::vector<std::thread> workers;
    UnboundedBlockingMPMCQueue<Job> jobs;
    ResourceCounter outstanding_work;
};

