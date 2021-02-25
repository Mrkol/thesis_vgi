#include "StaticThreadPool.hpp"

StaticThreadPool::StaticThreadPool(std::size_t size)
{
    workers.reserve(size);
    for (std::size_t i = 0; i < size; ++i)
    {
        workers.emplace_back([this]()
            {
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
