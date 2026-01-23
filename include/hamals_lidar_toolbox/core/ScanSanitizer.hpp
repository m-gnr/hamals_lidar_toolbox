#pragma once

#include "hamals_lidar_toolbox/core/ScanData.hpp"

namespace hamals_lidar_toolbox
{
namespace core
{

/**
 * @brief Ham LiDAR tarama verisini temizler ve normalize eder.
 *
 * ScanSanitizer, ScanData'yi sonraki isleme asamalari
 * (segmentasyon, analiz, tespit) icin guvenli ve tutarli hale getirir.
 *
 * Yapmaz:
 *  - engel tespiti yapmaz
 *  - segmentasyon yapmaz
 *  - konfigurasyon dosyasi okumaz
 *  - ROS'a bagimli degildir
 */
class ScanSanitizer
{
public:
    /**
     * @brief ScanSanitizer olusturur
     *
     * @param min_range Gecerli minimum mesafe (metre)
     * @param max_range Gecerli maksimum mesafe (metre)
     *
     * @throws std::invalid_argument Parametreler gecersizse
     */
    ScanSanitizer(double min_range, double max_range);

    /**
     * @brief Ham tarama verisini temizler
     *
     * - NaN degerleri max_range'e sabitlenir
     * - Sonsuz degerler max_range'e sabitlenir
     * - min_range altindaki degerler min_range'e sabitlenir
     * - max_range ustundeki degerler max_range'e sabitlenir
     *
     * @param input Ham ScanData
     * @return Yeni, temizlenmis ScanData ornegi
     */
    ScanData sanitize(const ScanData& input) const;

private:
    double min_range_;
    double max_range_;
};

} // namespace core
} // namespace hamals_lidar_toolbox
