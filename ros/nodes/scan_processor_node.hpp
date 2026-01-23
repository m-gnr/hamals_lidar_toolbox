#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "../adapters/LaserScanAdapter.hpp"


#include "hamals_lidar_toolbox/core/ScanSanitizer.hpp"
#include "hamals_lidar_toolbox/core/ScanSegmenter.hpp"
#include "hamals_lidar_toolbox/core/ScanMetrics.hpp"
#include "hamals_lidar_toolbox/core/ObstacleDetector.hpp"


#include <memory>

/**
 * @brief /scan verisini core pipeline'a baglayan ROS2 node'u
 *
 * Bu node:
 *  - /scan topic'ini dinler
 *  - LaserScanAdapter ile ROS mesajini core ScanData'ya cevirir
 *  - Core pipeline'i calistirir
 *  - Engel durumunu publish eder
 *
 * Bu sinif:
 *  - Algoritma icermez
 *  - Sadece orkestrasyon yapar
 */
class ScanProcessorNode : public rclcpp::Node
{
public:
    /**
     * @brief ScanProcessorNode constructor
     */
    explicit ScanProcessorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    std::vector<hamals_lidar_toolbox::core::ScanSegmenter::Region>
    createDefaultRegions() const;
    /**
     * @brief /scan callback fonksiyonu
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // =========================
    // ROS iletisim bilesenleri
    // =========================

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    // (ileride eklenecek)
    // rclcpp::Publisher<...>::SharedPtr obstacle_publisher_;

    // =========================
    // Core pipeline bilesenleri
    // =========================

    hamals_lidar_toolbox::core::ScanSanitizer sanitizer_;
    hamals_lidar_toolbox::core::ScanSegmenter segmenter_;
    hamals_lidar_toolbox::core::ObstacleDetector obstacle_detector_;

};
