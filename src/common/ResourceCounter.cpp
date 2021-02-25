#include "ResourceCounter.hpp"

void ResourceCounter::borrow_resource()
{
    std::lock_guard lock{mtx};
    ++borrowed;
}

void ResourceCounter::return_resource()
{
    std::lock_guard lock{mtx};
    --borrowed;
    if (borrowed == 0)
    {
        all_returned.notify_all();
    }
}

void ResourceCounter::wait_all_returned()
{
    std::unique_lock lock{mtx};
    while (borrowed > 0)
    {
        all_returned.wait(lock);
    }
}
