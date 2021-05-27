#pragma once

#include <ios>
#include <unordered_map>
#include <filesystem>


template<class S>
class FileStreamPool
{
public:
    explicit FileStreamPool(std::size_t size)
        : size_{size}
    {
    }

    S& get(std::filesystem::path path, std::ios_base::openmode mode)
    {
        auto it = streams_.find(path.string());

        if (it == streams_.end())
        {
            if (streams_.size() >= size_)
            {
                auto m = std::min_element(streams_.begin(), streams_.end(),
                    [](const auto& a, const auto& b)
                    {
                        return a.second.generation < b.second.generation;
                    });
                streams_.erase(m);
            }

            it = streams_.emplace(path.string(), Stream{0, std::make_unique<S>(path, mode)}).first;
        }

        it->second.generation = generation_++;

        return *it->second.stream.get();
    }


private:
    std::size_t size_;

    struct Stream
    {
        std::size_t generation;
        std::unique_ptr<S> stream;
    };
    std::unordered_map<std::string, Stream> streams_;

    std::size_t generation_{0};
};
