#pragma once

#include "hamals_lidar_toolbox/core/ScanData.hpp"

#include <vector>
#include <unordered_map>
#include <string>

namespace hamals_lidar_toolbox
{
namespace core
{

/**
 * @brief LiDAR tarama verisini açısal bölgelere ayırır.
 *
 * ScanSegmenter, temizlenmiş bir ScanData nesnesini alır ve
 * her ölçümün açısına bakarak hangi bölgeye (front, left, right, rear)
 * ait olduğunu belirler.
 *
 * Bu sınıf:
 *  - Mesafe hesaplamaz
 *  - Engel tespiti yapmaz
 *  - Karar vermez
 *  - ROS bağımlılığı içermez
 *
 * Sadece index -> bölge eşlemesi üretir.
 */
class ScanSegmenter
{
public:
    /**
     * @brief Tek bir bölge tanımını temsil eder
     *
     * Açı aralığı radyan cinsindendir.
     */
    struct Region
    {
        std::string name;   ///< Bölge adı (ör: "front")
        double min_angle;   ///< Bölgenin minimum açısı (radyan)
        double max_angle;   ///< Bölgenin maksimum açısı (radyan)
    };

    /**
     * @brief ScanSegmenter oluşturur
     *
     * @param regions Kullanılacak bölge tanımları
     *
     * Not:
     *  - Açıların normalize edilmiş olması beklenir
     *  - Bölge çakışmaları kullanıcı sorumluluğundadır
     */
    explicit ScanSegmenter(const std::vector<Region>& regions);

    /**
     * @brief ScanData verisini bölgelere ayırır
     *
     * Her bölge için, o bölgeye düşen ölçümlerin index listesi döndürülür.
     *
     * @param scan Temizlenmiş ScanData
     * @return Bölge adı -> index listesi haritası
     */
    std::unordered_map<std::string, std::vector<std::size_t>>
    segment(const ScanData& scan) const;

private:
    std::vector<Region> regions_;
};

} // namespace core
} // namespace hamals_lidar_toolbox
