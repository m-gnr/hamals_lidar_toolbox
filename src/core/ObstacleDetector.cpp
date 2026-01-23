#include "hamals_lidar_toolbox/core/ObstacleDetector.hpp"

#include <stdexcept>

namespace hamals_lidar_toolbox
{
namespace core
{

ObstacleDetector::ObstacleDetector(double danger_distance)
    : danger_distance_(danger_distance)
{
    // Tehlike mesafesi pozitif olmak zorundadir
    if (danger_distance_ <= 0.0)
    {
        throw std::invalid_argument(
            "ObstacleDetector: danger_distance must be > 0");
    }
}

std::unordered_map<std::string, bool>
ObstacleDetector::detect(
    const std::unordered_map<std::string, RegionMetrics>& metrics) const
{
    std::unordered_map<std::string, bool> result;

    // Tum bolgeler icin engel var mi kontrol ediyoruz
    for (const auto& pair : metrics)
    {
        const std::string& region_name = pair.first;
        const RegionMetrics& region_metrics = pair.second;

        bool obstacle_detected = false;

        // Engel vardir EGER:
        // - Bolge bos degilse (count > 0)
        // - En yakin mesafe danger_distance'dan kucukse
        if (region_metrics.count > 0 &&
            region_metrics.min_distance < danger_distance_)
        {
            obstacle_detected = true;
        }

        result[region_name] = obstacle_detected;
    }

    return result;
}

} // namespace core
} // namespace hamals_lidar_toolbox
