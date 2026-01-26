#pragma once

#include "hamals_lidar_toolbox/core/ScanData.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

namespace hamals_lidar_toolbox
{
namespace ros
{
namespace adapters
{

/**
 * @brief ROS LaserScan mesajini core ScanData nesnesine donusturur.
 *
 * LaserScanAdapter:
 *  - sensor_msgs::msg::LaserScan alir
 *  - Core katmaninin anlayacagi ScanData nesnesini uretir
 *
 * Bu sinif:
 *  - Filtreleme yapmaz
 *  - NaN / inf temizlemez
 *  - Engel tespiti yapmaz
 *  - Segmentasyon yapmaz
 *  - Config okumaz
 *
 * Tum is mantigi core pipeline icindedir.
 */
class LaserScanAdapter
{
public:
    /**
     * @brief LaserScan mesajini ScanData'ya donusturur
     *
     * @param msg ROS LaserScan mesaji
     * @return Olusturulan ScanData nesnesi
     *
     * @throws std::invalid_argument Mesaj gecersizse
     */
    static core::ScanData
    fromRosMessage(const sensor_msgs::msg::LaserScan& msg);
};

} // namespace adapters
} // namespace ros
} // namespace hamals_lidar_toolbox
