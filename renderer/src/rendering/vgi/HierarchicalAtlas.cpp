#include "HierarchicalAtlas.hpp"

#include <unordered_set>
#include <fstream>
#include <utility>

#include "Utility.hpp"


GeometryImage::GeometryImage(const std::filesystem::path& path)
{
    std::ifstream in{path, std::ios_base::binary};

    data.resize(file_size(path) / sizeof(Eigen::Vector3f));
    in.read(reinterpret_cast<char*>(data.data()),
        static_cast<std::streamsize>(data.size() * sizeof(Eigen::Vector3f)));

    width = static_cast<std::size_t>(std::sqrt(data.size()));
}

Eigen::Vector3f& GeometryImage::operator ()(std::size_t x, std::size_t y)
{
    AD_HOC_ASSERT(x < width && y < width, "out of bounds");
    return data[y*width + x];
}

const Eigen::Vector3f& GeometryImage::operator ()(std::size_t x, std::size_t y) const
{
    return const_cast<GeometryImage&>(*this).operator ()(x, y);
}

GeometryImage GeometryImage::upscale(std::size_t newsize) const
{
    GeometryImage result;

    result.data.resize(newsize * newsize);
    result.width = newsize;

    static constexpr auto lerp = []<class T>(T c1, T c2, float k)
        {
            return (1 - k)*c1 + k*c2;
        };
    static constexpr auto blerp = []<class T>(T c11, T c21, T c12, T c22, float kx, float ky)
        {
            return lerp(lerp(c11, c21, kx), lerp(c12, c22, kx), ky);
        };


    for (std::size_t x = 0; x < newsize; ++x)
    {
        for (std::size_t y = 0; y < newsize; ++y)
        {
            float x_ = float(x * width) / float(newsize);
            float y_ = float(y * width) / float(newsize);

            auto xi = std::size_t(x_);
            auto yi = std::size_t(y_);

            result(x, y) =
                blerp
                ( operator ()(xi,     yi)
                , operator ()(xi + 1, yi)
                , operator ()(xi,     yi + 1)
                , operator ()(xi + 1, yi + 1)
                , x_ - float(xi)
                , y_ - float(yi)
                );
        }
    }

    return result;
}

QuadtreeNode::QuadtreeNode(QuadtreeNode::CreateInfo info)
    : children{nullptr}
    , size{info.size}
    , offset{info.offset}
    , bounding_box_start{Eigen::Vector3f::Constant(std::numeric_limits<float>::max())}
    , bounding_box_end{Eigen::Vector3f::Constant(-std::numeric_limits<float>::max())}
    , min_tessellation{info.levels_left}
    , max_tessellation{info.levels_left + info.GIs.size() - 1}
    , mip_errors(info.GIs.size(), 0)
{
    if (info.levels_left > 0)
    {
        float size_ = size/2;
        std::array offsets{
            Eigen::Vector2f{0, 0},
            Eigen::Vector2f{size_, 0},
            Eigen::Vector2f{0, size_},
            Eigen::Vector2f{size_, size_},
        };

        for (std::size_t i = 0; i < 4; ++i)
        {
            children[i] = std::make_unique<QuadtreeNode>(CreateInfo{
                info.levels_left - 1,
                size_,
                offset + offsets[i],
                info.GIs,
                info.upsampled_GIs
            });
            children[i]->parent = this;

            for (std::size_t j = 0; j < mip_errors.size(); ++j)
            {
                mip_errors[j] = std::max(mip_errors[j], children[i]->mip_errors[j]);
            }
            bounding_box_start = bounding_box_start.cwiseMin(children[i]->bounding_box_start);
            bounding_box_end = bounding_box_end.cwiseMax(children[i]->bounding_box_end);
        }

    }
    else
    {
        using IntVec = Eigen::Matrix<std::size_t, 2, 1>;

        for (std::size_t i = 1; i < info.GIs.size(); ++i)
        {
            auto& gi = info.upsampled_GIs[i - 1];
            auto& original_gi = info.GIs[0];

            auto subregion_size = std::size_t(float(original_gi.size()) * size);
            auto subregion_offset = (float(original_gi.size()) * offset).cast<std::size_t>().eval();

            float& error = mip_errors[i];
            for (std::size_t x = 0; x < subregion_size; ++x)
            {
                for (std::size_t y = 0; y < subregion_size; ++y)
                {
                    auto p = gi(subregion_offset.x() + x, subregion_offset.y() + y);
                    auto po = original_gi(subregion_offset.x() + x, subregion_offset.y() + y);
                    error = std::max(error, (p - po).norm());

                    bounding_box_start = bounding_box_start.cwiseMin(po);
                    bounding_box_end = bounding_box_end.cwiseMax(po);
                }
            }
        }
    }
}

PatchRoot::PatchRoot(CreateInfo info)
{
    gis.reserve(info.max_mip - info.min_mip + 1);
    for (std::size_t i = info.min_mip; i <= info.max_mip; ++i)
    {
        gis.emplace_back(
            info.images_folder / "positions" / (std::string{info.name_prefix} + ":" + std::to_string(i))
        );
    }

    std::vector<GeometryImage> upsampled;
    upsampled.reserve(gis.size() - 1);

    for (std::size_t i = 1; i < gis.size(); ++i)
    {
        upsampled.emplace_back(gis[i].upscale(gis[0].size()));
    }

    root = std::make_unique<QuadtreeNode>(QuadtreeNode::CreateInfo{
        info.min_mip,
        1.f, Eigen::Vector2f::Zero(),
        gis, upsampled
    });

    {
        auto& gi = gis[0];
        auto sz = gi.size() - 1;
        max_res_corners =
            { gi(0, 0)
            , gi(0, sz)
            , gi(sz, sz)
            , gi(sz, 0)
            };
    }
}

HierarchicalAtlas::HierarchicalAtlas(std::filesystem::path images_folder)
{
    std::unordered_set<std::string> prefixes;
    min_mip = 1000; // 2^1000 is a lot
    max_mip = 0;
    for (auto& entry : std::filesystem::directory_iterator(images_folder / "positions"))
    {
        auto name = entry.path().filename().string();
        auto pos = name.rfind(':');
        prefixes.emplace(name.substr(0, pos));
        std::size_t mip = std::stoul(name.substr(pos + 1));
        min_mip = std::min(min_mip, mip);
        max_mip = std::max(max_mip, mip);
    }

    patches.reserve(prefixes.size());
    for (auto& prefix : prefixes)
    {
        patches.emplace_back(PatchRoot::CreateInfo{images_folder, prefix, min_mip, max_mip});
    }

    // O(n^2) is ok here as n < 100
    for (auto& patch1 : patches)
    {
        for (auto& patch2 : patches)
        {
            if (&patch1 == &patch2)
            {
                continue;
            }

            if ((patch1.max_res_corners[0] - patch2.max_res_corners[3]).norm() < 1e-9
                && (patch1.max_res_corners[1] - patch2.max_res_corners[2]).norm() < 1e-9)
            {
                patch1.neighbors[2] = &patch2;
                patch2.neighbors[0] = &patch1;
            }

            if ((patch1.max_res_corners[1] - patch2.max_res_corners[0]).norm() < 1e-9
                && (patch1.max_res_corners[2] - patch2.max_res_corners[3]).norm() < 1e-9)
            {
                patch1.neighbors[1] = &patch2;
                patch2.neighbors[3] = &patch1;
            }
        }
    }
}

HierarchyCut HierarchicalAtlas::default_cut()
{
    HierarchyCut result;

    for (std::size_t i = 0; i < patches.size(); ++i)
    {
        result.elements.emplace(patches[i].root.get(),
            HierarchyCut::CutElement{i, min_mip, {min_mip}, &patches[i]});
    }

    result.split(patches[6].root.get());

    return result;
}

void HierarchyCut::split(QuadtreeNode* node)
{
    auto it = elements.find(node);

    if (it == elements.end() || node->children[0] == nullptr)
    {
        return;
    }

    CutElement data = it->second;
    elements.erase(it);

    --data.mip;
    for (auto& child : node->children)
    {
        elements.emplace(child.get(), data);
    }
}

std::vector<std::pair<QuadtreeNode*, HierarchyCut::CutElement>> HierarchyCut::dump() const
{
    std::vector<std::pair<QuadtreeNode*, HierarchyCut::CutElement>> result;
    for (const auto& pair : elements)
    {
        result.emplace_back(pair);
    }
    return result;
}

const HierarchyCut::CutElement& HierarchyCut::get_data(QuadtreeNode* node) const
{
    return elements.at(node);
}

void HierarchyCut::set_mip(QuadtreeNode* node, std::size_t mip)
{
    auto& data = elements.at(node);
    if (data.mip == mip)
    {
        return;
    }

    data.mip = mip;
    // TODO: recalculate side mips
}
