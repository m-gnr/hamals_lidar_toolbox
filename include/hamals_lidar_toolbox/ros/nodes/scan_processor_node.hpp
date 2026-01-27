#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "hamals_lidar_toolbox/ros/adapters/LaserScanAdapter.hpp"

#include "hamals_lidar_toolbox/core/ScanSanitizer.hpp"
#include "hamals_lidar_toolbox/core/ScanSegmenter.hpp"
#include "hamals_lidar_toolbox/core/ScanMetrics.hpp"
#include "hamals_lidar_toolbox/core/ObstacleDetector.hpp"
#include "hamals_lidar_msgs/msg/obstacle_state.hpp"

#include "hamals_lidar_toolbox/ros/rviz/RvizDebugPublisher.hpp"

#include <memory>
#include <vector>

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
    explicit ScanProcessorNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    /**
     * @brief Config parametrelerinden bolge tanimlarini olusturur
     */
    std::vector<hamals_lidar_toolbox::core::ScanSegmenter::Region>
    createRegionsFromParams() const;

    /**
     * @brief /scan callback fonksiyonu
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // =========================
    // ROS iletisim bilesenleri
    // =========================

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_subscriber_;

    rclcpp::Publisher<
        hamals_lidar_msgs::msg::ObstacleState
    >::SharedPtr obstacle_state_pub_;

    // =========================
    // Core pipeline bilesenleri
    // (parametre okunduktan sonra olusturulur)
    // =========================

    std::unique_ptr<hamals_lidar_toolbox::core::ScanSanitizer> sanitizer_;
    std::unique_ptr<hamals_lidar_toolbox::core::ScanSegmenter> segmenter_;
    std::unique_ptr<hamals_lidar_toolbox::core::ObstacleDetector> obstacle_detector_;

    // =========================
    // RViz Debug (FAZ 4)
    // =========================

    bool debug_rviz_enabled_{false};

    std::unique_ptr<
        hamals_lidar_toolbox::ros::rviz::RvizDebugPublisher
    > rviz_debug_;
};
