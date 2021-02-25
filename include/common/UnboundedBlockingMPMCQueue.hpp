#pragma once

#include <mutex>
#include <condition_variable>
#include <queue>
#include <optional>


template<class T>
class UnboundedBlockingMPMCQueue
{
public:
    void put(T t)
    {
        std::lock_guard lock{mtx};
        if (closed)
        {
            throw std::logic_error("Trying to put into a closed queue!");
        }
        impl.push(std::move(t));
        is_empty.notify_one();
    }

    std::optional<T> take()
    {
        std::unique_lock lock{mtx};
        while (impl.empty())
        {
            if (closed)
            {
                return {};
            }
            is_empty.wait(lock);
        }
        std::optional<T> result{std::move(impl.front())};
        impl.pop();
        // NRVO
        return result;
    }

    void close()
    {
        std::lock_guard lock{mtx};
        if (closed)
        {
            throw std::logic_error{"Trying to double close a queue!"};
        }
        closed = true;
    }

private:
    std::queue<T> impl;
    std::mutex mtx;
    std::condition_variable is_empty;
    bool closed{false};
};
