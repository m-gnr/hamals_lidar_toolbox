#include "LaserScanAdapter.hpp"

#include <stdexcept>

namespace hamals_lidar_toolbox
{
namespace ros
{
namespace adapters
{

core::ScanData
LaserScanAdapter::fromRosMessage(const sensor_msgs::msg::LaserScan& msg)
{
    // Temel alan kontrolleri
    if (msg.ranges.empty())
    {
        throw std::invalid_argument(
            "LaserScanAdapter: LaserScan message contains empty ranges");
    }

    if (msg.angle_increment == 0.0f)
    {
        throw std::invalid_argument(
            "LaserScanAdapter: angle_increment must be non-zero");
    }

    // ROS zaman damgasini saniye cinsinden double'a cevir
    double timestamp_sec =
        static_cast<double>(msg.header.stamp.sec) +
        static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

    // ScanData nesnesini olustur
    return core::ScanData(
        msg.ranges,           // mesafeler
        msg.angle_min,        // baslangic acisi
        msg.angle_increment,  // aci artisi
        timestamp_sec         // zaman damgasi (saniye)
    );
}

} // namespace adapters
} // namespace ros
} // namespace hamals_lidar_toolbox
