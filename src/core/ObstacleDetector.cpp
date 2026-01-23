#include "hamals_lidar_toolbox/core/ObstacleDetector.hpp"

namespace hamals_lidar_toolbox
{
namespace core
{

void ObstacleDetector::setDangerDistance(double distance)
{
    danger_distance_ = distance;
}

ObstacleMap ObstacleDetector::detect(
    const std::unordered_map<std::string, RegionMetrics>& metrics) const
{
    ObstacleMap result;

    for (const auto& [region, region_metrics] : metrics)
    {
        ObstacleState state;
        state.min_distance = region_metrics.min_distance;
        state.has_obstacle =
            (region_metrics.count > 0 &&
             region_metrics.min_distance < danger_distance_);

        result[region] = state;
    }

    return result;
}

} // namespace core
} // namespace hamals_lidar_toolbox
