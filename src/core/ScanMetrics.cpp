#include "hamals_lidar_toolbox/core/ScanMetrics.hpp"

#include <limits>

namespace hamals_lidar_toolbox
{
namespace core
{

std::unordered_map<std::string, RegionMetrics>
ScanMetrics::compute(
    const ScanData& scan,
    const std::unordered_map<std::string, std::vector<std::size_t>>& segments)
{
    std::unordered_map<std::string, RegionMetrics> result;

    for (const auto& pair : segments)
    {
        const std::string& region_name = pair.first;
        const std::vector<std::size_t>& indices = pair.second;

        RegionMetrics metrics;

        if (indices.empty())
        {
            result[region_name] = metrics;
            continue;
        }

        double sum = 0.0;
        double min_val = std::numeric_limits<double>::infinity();

        for (std::size_t idx : indices)
        {
            double distance = static_cast<double>(scan.ranges()[idx]);

            if (distance < min_val)
            {
                min_val = distance;
            }

            sum += distance;
        }

        metrics.count = indices.size();
        metrics.min_distance = min_val;
        metrics.mean_distance = sum / static_cast<double>(indices.size());

        result[region_name] = metrics;
    }

    return result;
}

} // namespace core
} // namespace hamals_lidar_toolbox
