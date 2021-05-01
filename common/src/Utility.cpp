#include "Utility.hpp"

#include <iostream>
#include <mutex>

static std::mutex mutex;

void detail::panic(detail::SourceLocation loc, std::string_view message)
{
    {
        std::lock_guard lock{mutex};
        std::cerr << "Panicked at ";
        std::cout << loc.file << ":" << loc.line << " (" << loc.function << ") with error" << "\n";
        std::cout << message << std::endl;
    }
    std::abort();
}
