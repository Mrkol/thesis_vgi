#pragma once

#include <mutex>
#include <condition_variable>

class ResourceCounter
{
public:
    void borrow_resource();
    void return_resource();

    void wait_all_returned();

private:
    std::size_t borrowed{};
    std::mutex mtx;
    std::condition_variable all_returned;
};
