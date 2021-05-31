#include "GeometryImage.hpp"

#include "Utility.hpp"
#include <utility>
#include <fstream>

GeometryImage::GeometryImage(const std::filesystem::path& path)
{
    std::ifstream in{path, std::ios_base::binary};

    data.resize(file_size(path) / sizeof(data[0]));
    in.read(reinterpret_cast<char*>(data.data()),
        static_cast<std::streamsize>(data.size() * sizeof(data[0])));

    width = static_cast<std::size_t>(std::sqrt(data.size()));
}

Eigen::Vector3f& GeometryImage::operator ()(std::size_t x, std::size_t y)
{
    AD_HOC_ASSERT(x < width && y < width, "out of bounds");
    return data[y*width + x].position;
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