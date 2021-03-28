#include "StaticThreadPool.hpp"


static thread_local std::size_t current_thread_idx = StaticThreadPool::THREAD_NONE;

StaticThreadPool::StaticThreadPool(std::size_t size)
{
    workers.reserve(size);
    for (std::size_t i = 0; i < size; ++i)
    {
        workers.emplace_back([this, i]()
            {
                current_thread_idx = i;
                while (auto job = jobs.take())
                {
                    (*job)();
                    outstanding_work.return_resource();
                }
            });
    }
}

void StaticThreadPool::submit(StaticThreadPool::Job job)
{
    outstanding_work.borrow_resource();
    jobs.put(std::move(job));
}

StaticThreadPool::~StaticThreadPool()
{
    jobs.close();
    outstanding_work.wait_all_returned();
    for (auto& worker : workers)
    {
        worker.join();
    }
}

std::size_t StaticThreadPool::current_thread_index()
{
    return current_thread_idx;
}
