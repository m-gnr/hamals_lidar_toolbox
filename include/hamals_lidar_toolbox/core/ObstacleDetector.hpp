#pragma once

#include "hamals_lidar_toolbox/core/ScanMetrics.hpp"

#include <unordered_map>
#include <string>

namespace hamals_lidar_toolbox
{
namespace core
{

/**
 * @brief Bölgesel metriklere bakarak engel var/yok tespiti yapar.
 *
 * ObstacleDetector sınıfı:
 *  - Sayısal ölçüm (RegionMetrics) alır
 *  - Basit bir eşik kuralı uygular
 *  - Her bölge için bool sonuç üretir
 *
 * Bu sınıf:
 *  - ROS bilmez
 *  - /scan bilmez
 *  - Yön seçmez
 *  - Hareket üretmez
 */
class ObstacleDetector
{
public:
    /**
     * @brief ObstacleDetector oluşturur
     *
     * @param danger_distance Engel olarak kabul edilen minimum mesafe (metre)
     *
     * @throws std::invalid_argument danger_distance <= 0 ise
     */
    explicit ObstacleDetector(double danger_distance);

    /**
     * @brief Bölgelere göre engel tespiti yapar
     *
     * Engel vardır EĞER:
     *  - region.min_distance < danger_distance
     *
     * @param metrics Bölge adı -> RegionMetrics haritası
     * @return Bölge adı -> engel var mı (true/false) haritası
     */
    std::unordered_map<std::string, bool>
    detect(const std::unordered_map<std::string, RegionMetrics>& metrics) const;

private:
    /// Tehlikeli kabul edilen mesafe eşiği
    double danger_distance_;
};

} // namespace core
} // namespace hamals_lidar_toolbox
