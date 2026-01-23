#include <iostream>
#include <vector>
#include <cmath>

#include "hamals_lidar_toolbox/core/ScanData.hpp"
#include "hamals_lidar_toolbox/core/ScanSanitizer.hpp"
#include "hamals_lidar_toolbox/core/ScanSegmenter.hpp"
#include "hamals_lidar_toolbox/core/ScanMetrics.hpp"
#include "hamals_lidar_toolbox/core/ObstacleDetector.hpp"

using namespace hamals_lidar_toolbox::core;

int main()
{
    std::cout << "=== Core pipeline test ===" << std::endl;

    // 1️⃣ Fake LiDAR scan (360 derece, 1 derece resolution)
    std::vector<float> ranges(360, 5.0f);

    // Front bolgesine yakin bir engel koyalim
    ranges[0] = 0.3f;
    ranges[359] = 0.4f;

    ScanData scan(
        ranges,
        -M_PI,                  // angle_min
        2.0 * M_PI / 360.0,     // angle_increment
        123.456                 // timestamp
    );

    // 2️⃣ Sanitizer
    ScanSanitizer sanitizer(0.1, 10.0);
    ScanData clean_scan = sanitizer.sanitize(scan);

    // 3️⃣ Segmenter
    std::vector<ScanSegmenter::Region> regions = {
        {"front", -15.0 * M_PI / 180.0,  15.0 * M_PI / 180.0},
        {"left",   15.0 * M_PI / 180.0,  90.0 * M_PI / 180.0},
        {"right", -90.0 * M_PI / 180.0, -15.0 * M_PI / 180.0},
        {"rear",   90.0 * M_PI / 180.0, -90.0 * M_PI / 180.0} // wrap-around
    };

    ScanSegmenter segmenter(regions);
    auto segments = segmenter.segment(clean_scan);

    // 4️⃣ Metrics
    auto metrics = ScanMetrics::compute(clean_scan, segments);

    // 5️⃣ Obstacle detection
    ObstacleDetector detector(0.5);
    auto obstacles = detector.detect(metrics);

    // 6️⃣ Sonuclari yazdir
    for (const auto& pair : obstacles)
    {
        std::cout << "Region: " << pair.first
                  << " | obstacle: "
                  << (pair.second ? "YES" : "NO")
                  << " | min_distance: "
                  << metrics[pair.first].min_distance
                  << std::endl;
    }

    return 0;
}
