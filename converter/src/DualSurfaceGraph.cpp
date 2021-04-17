#include "DualSurfaceGraph.hpp"

#include <stack>


std::vector<std::size_t> DualSurfaceGraph::paint_quads(
    const std::vector<ThickTriangle>& patch, const std::unordered_set<SymmetricEdge>& banned) const
{
    std::vector<std::size_t> triangle_colors;
    triangle_colors.resize(patch.size(), COLOR_NONE);

    std::size_t current_color = 0;
    for (std::size_t i = 0; i < patch.size(); ++i)
    {
        if (triangle_colors[i] != COLOR_NONE)
        {
            continue;
        }

        std::stack<std::size_t> stack;
        stack.push(i);

        while (!stack.empty())
        {
            std::size_t idx = stack.top();
            stack.pop();

            if (triangle_colors[idx] != COLOR_NONE)
            {
                continue;
            }

            triangle_colors[idx] = current_color;

            auto edges = triangle_edges(patch[idx]);
            for (auto& e : edges)
            {
                auto next = find_not(e, idx);
                if (!banned.contains(e) && next != DualSurfaceGraph::INVALID
                    && triangle_colors[next] == COLOR_NONE)
                {
                    stack.push(next);
                }
            }
        }

        ++current_color;
    }

    return triangle_colors;
}
