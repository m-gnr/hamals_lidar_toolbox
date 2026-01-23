#include "hamals_lidar_toolbox/core/ScanSanitizer.hpp"

#include <stdexcept>
#include <cmath>        // std::isfinite

namespace hamals_lidar_toolbox
{
namespace core
{

ScanSanitizer::ScanSanitizer(double min_range, double max_range)
    : min_range_(min_range),
      max_range_(max_range)
{
    // Minimum mesafe negatif olamaz
    if (min_range_ < 0.0)
    {
        throw std::invalid_argument(
            "ScanSanitizer: min_range 0'dan kucuk olamaz");
    }

    // Maksimum mesafe minimumdan buyuk olmalidir
    if (max_range_ <= min_range_)
    {
        throw std::invalid_argument(
            "ScanSanitizer: max_range min_range'den buyuk olmalidir");
    }
}

ScanData ScanSanitizer::sanitize(const ScanData& input) const
{
    // Yeni ranges vektoru olusturuyoruz (boyut korunacak)
    std::vector<float> clean_ranges;
    clean_ranges.reserve(input.size());

    // Tum olcumleri tek tek kontrol ediyoruz
    for (float value : input.ranges())
    {
        // NaN veya sonsuz deger kontrolu
        if (!std::isfinite(value))
        {
            // Gecersiz degerleri max_range olarak kabul ediyoruz
            clean_ranges.push_back(static_cast<float>(max_range_));
            continue;
        }

        // Minimum mesafeden kucukse min_range'e cek
        if (value < min_range_)
        {
            clean_ranges.push_back(static_cast<float>(min_range_));
            continue;
        }

        // Maksimum mesafeden buyukse max_range'e cek
        if (value > max_range_)
        {
            clean_ranges.push_back(static_cast<float>(max_range_));
            continue;
        }

        // Gecerli aralikta ise oldugu gibi ekle
        clean_ranges.push_back(value);
    }

    // Temizlenmis yeni ScanData nesnesi olusturuyoruz
    return ScanData(
        std::move(clean_ranges),
        input.angleMin(),
        input.angleIncrement(),
        input.timestamp()
    );
}

} // namespace core
} // namespace hamals_lidar_toolbox
