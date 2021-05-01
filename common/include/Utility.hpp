#pragma once

#include <string_view>


namespace detail
{

struct SourceLocation
{
    std::string_view file;
    std::string_view function;
    std::size_t line;
};

[[noreturn]] void panic(SourceLocation loc, std::string_view message);

}

// Mostly copied from gitlab.com/Lipovsky/wheels

#ifdef _MSC_VER
#define __PRETTY_FUNCTION__ __FUNCSIG__
#endif

#define AD_HOC_HERE detail::SourceLocation{__FILE__, __PRETTY_FUNCTION__, __LINE__}

#define AD_HOC_PANIC(msg)        \
    do                           \
    {                            \
        panic(AD_HOC_HERE, msg); \
    } while (false)
