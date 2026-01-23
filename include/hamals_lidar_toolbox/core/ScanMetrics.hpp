#pragma once

#include "hamals_lidar_toolbox/core/ScanData.hpp"

#include <unordered_map>
#include <vector>
#include <string>
#include <cstddef>
#include <limits>

namespace hamals_lidar_toolbox
{
namespace core
{

/**
 * @brief Tek bir bölgeye ait sayısal ölçümleri tutar.
 *
 * Bu yapı, ScanSegmenter sonucunda elde edilen
 * bir bölgenin (front, left, right, rear) istatistiksel
 * özetini temsil eder.
 */
struct RegionMetrics
{
    std::size_t count = 0;                ///< Bu bölgedeki ölçüm sayısı
    double min_distance = std::numeric_limits<double>::infinity();   ///< En yakın mesafe
    double mean_distance = std::numeric_limits<double>::infinity();  ///< Ortalama mesafe
};

/**
 * @brief Bölgelere ayrılmış LiDAR verisi üzerinden metrikler hesaplar.
 *
 * ScanMetrics sınıfı:
 *  - Engel tespiti yapmaz
 *  - Eşik kullanmaz
 *  - Karar vermez
 *  - ROS bağımlılığı içermez
 *
 * Sadece sayısal ölçüm üretir.
 */
class ScanMetrics
{
public:
    /**
     * @brief Bölgesel metrikleri hesaplar
     *
     * @param scan Temizlenmiş ScanData
     * @param segments Bölge adı -> index listesi haritası
     *
     * @return Bölge adı -> RegionMetrics haritası
     */
    static std::unordered_map<std::string, RegionMetrics>
    compute(
        const ScanData& scan,
        const std::unordered_map<std::string, std::vector<std::size_t>>& segments
    );
};

} // namespace core
} // namespace hamals_lidar_toolbox
