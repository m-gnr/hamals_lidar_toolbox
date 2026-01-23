#include "hamals_lidar_toolbox/core/ScanSegmenter.hpp"

#include <cmath>        // std::fmod
#include <stdexcept>

namespace hamals_lidar_toolbox
{
namespace core
{

// Aciyi [-pi, +pi] araligina normalize eder
static double normalizeAngle(double angle)
{
    // fmod ile 2*pi moduna sokuyoruz
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0)
    {
        angle += 2.0 * M_PI;
    }
    return angle - M_PI;
}

ScanSegmenter::ScanSegmenter(const std::vector<Region>& regions)
    : regions_(regions)
{
    // En az bir bolge tanimi olmasi gerekir
    if (regions_.empty())
    {
        throw std::invalid_argument(
            "ScanSegmenter: region list cannot be empty");
    }
}

std::unordered_map<std::string, std::vector<std::size_t>>
ScanSegmenter::segment(const ScanData& scan) const
{
    std::unordered_map<std::string, std::vector<std::size_t>> result;

    // Tum bolgeler icin bos vektorleri onceden olusturuyoruz
    for (const auto& region : regions_)
    {
        result[region.name] = {};
    }

    const double angle_min = scan.angleMin();
    const double angle_inc = scan.angleIncrement();
    const std::size_t count = scan.size();

    // Tum olcumleri index bazinda geziyoruz
    for (std::size_t i = 0; i < count; ++i)
    {
        // Index'ten aciyi hesapla
        double angle = angle_min + static_cast<double>(i) * angle_inc;

        // Aciyi normalize et
        double normalized_angle = normalizeAngle(angle);

        // Bu index hangi bolgeye dusuyor kontrol et
        for (const auto& region : regions_)
        {
            double min_a = region.min_angle;
            double max_a = region.max_angle;

            // Normal durum: min <= max
            if (min_a <= max_a)
            {
                if (normalized_angle >= min_a &&
                    normalized_angle <= max_a)
                {
                    result[region.name].push_back(i);
                }
            }
            else
            {
                // Wrap-around durumu (ornegin rear bolgesi)
                if (normalized_angle >= min_a ||
                    normalized_angle <= max_a)
                {
                    result[region.name].push_back(i);
                }
            }
        }
    }

    return result;
}

} // namespace core
} // namespace hamals_lidar_toolbox
