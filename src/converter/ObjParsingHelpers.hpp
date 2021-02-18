#pragma once

#include <charconv>
#include <optional>
#include <string_view>
#include <tuple>
#include <string>
#include <cmath>


using namespace std::literals::string_view_literals;

constexpr char DEFAULT_SPACES[] = " \r\t\n";

// (token, rest of string)
template<const char* SPACES = DEFAULT_SPACES>
inline std::pair<std::string_view, std::string_view> next_token(std::string_view what)
{
    auto token_start = what.find_first_not_of(SPACES);
    auto token_end = what.find_first_of(SPACES, token_start);
    if (token_start != std::string::npos)
    {
        if (token_end != std::string::npos)
        {
           return {what.substr(token_start, token_end), what.substr(token_end)};
        }

        return {what.substr(token_start), ""sv};
    }
    return {""sv, ""sv};
}

template<class T>
bool to_number(std::string_view view, T& result)
{
    if constexpr (std::is_integral_v<T>)
    {
        return std::from_chars(view.data(), view.data() + view.size(), result).ec == std::errc{};
    }
    else
    {
        char* end;
        // This is potentially dangerous. Why does linux not have from_chars for floats yet???!!!
        result = std::strtod(view.data(), &end);

        return std::isfinite(result) && end != view.data();
    }
}

template<class T, const char* SPACES = DEFAULT_SPACES>
inline std::optional<std::tuple<T, T, T>> try_get_triple(std::string_view line)
{
    auto[x_str, rest2] = next_token<SPACES>(line);
    auto[y_str, rest3] = next_token<SPACES>(rest2);
    auto[z_str, rest4] = next_token<SPACES>(rest3);

    std::tuple<T, T, T> result;

    if (!to_number<T>(x_str, std::get<0>(result))
        || !to_number<T>(y_str, std::get<1>(result))
        || !to_number<T>(z_str, std::get<2>(result)))
    {
        return {};
    }

    return {result};
}
