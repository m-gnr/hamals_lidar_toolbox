#pragma once

#include <vector>
#include <cstddef>

namespace hamals_lidar_toolbox
{
namespace core
{

class ScanData
{
public:
    /**
     * @brief Construct a ScanData object representing a single LiDAR scan frame
     *
     * @param ranges            Distance measurements (meters)
     * @param angle_min         Angle of the first measurement (radians)
     * @param angle_increment   Angular distance between measurements (radians)
     * @param timestamp         Timestamp of this scan (seconds)
     *
     * @throws std::invalid_argument if input data is invalid
     */
    ScanData(
        std::vector<float> ranges,
        double angle_min,
        double angle_increment,
        double timestamp
    );

    // --- Accessors (read-only) ---

    /// @return All distance measurements
    const std::vector<float>& ranges() const;

    /// @return Angle of the first measurement (radians)
    double angleMin() const;

    /// @return Angular distance between measurements (radians)
    double angleIncrement() const;

    /// @return Number of measurements
    std::size_t size() const;

    /// @return Timestamp of the scan (seconds)
    double timestamp() const;

private:
    // --- Internal data (immutable by design) ---

    std::vector<float> ranges_;
    double angle_min_;
    double angle_increment_;
    double timestamp_;
};

} // namespace core
} // namespace hamals_lidar_toolbox
