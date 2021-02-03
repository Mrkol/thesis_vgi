#pragma once

#include <charconv>
#include <optional>
#include <string_view>


using namespace std::literals::string_view_literals;

static constexpr auto SPACE_SYMBOLS = " \t\r\n";

// (token, rest of string)
inline std::pair<std::string_view, std::string_view> next_token(std::string_view what)
{
    auto token_start = what.find_first_not_of(SPACE_SYMBOLS);
    auto token_end = what.find_first_of(SPACE_SYMBOLS, token_start);
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

inline bool to_float(std::string_view view, float& result)
{
    return std::from_chars(view.data(), view.data() + view.size(), result).ec == std::errc{};
}

inline std::optional<std::tuple<float, float, float>> try_get_vertex(std::string_view line)
{
    auto[token, rest1] = next_token(line);

    if (token != "v") { return {}; }
    
    auto[x_str, rest2] = next_token(rest1);
    auto[y_str, rest3] = next_token(rest2);
    auto[z_str, rest4] = next_token(rest3);

    std::tuple<float, float, float> result;

    if (!to_float(x_str, std::get<0>(result))
        || !to_float(y_str, std::get<1>(result))
        || !to_float(z_str, std::get<2>(result)))
    {
        return {};
    }

    return {result};
}

