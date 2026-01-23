#include "scan_processor_node.hpp"
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;

ScanProcessorNode::ScanProcessorNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("scan_processor_node", options),
  sanitizer_(0.05, 30.0),                 // min_range, max_range
  segmenter_(createDefaultRegions())      // region tanimlari
{
    // Tehlike mesafesi (simdilik sabit)
    double danger_distance = 0.5;
    obstacle_detector_.setDangerDistance(danger_distance);

    // /scan subscriber
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&ScanProcessorNode::scanCallback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "ScanProcessorNode started.");
}

void ScanProcessorNode::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 1️⃣ ROS -> Core (adapter)
    hamals_lidar_toolbox::core::ScanData scan =
        hamals_lidar_toolbox::ros::adapters::LaserScanAdapter::fromRosMessage(*msg);

    // 2️⃣ Temizleme
    hamals_lidar_toolbox::core::ScanData clean_scan =
        sanitizer_.sanitize(scan);

    // 3️⃣ Segmentasyon
    auto segments = segmenter_.segment(clean_scan);

    // 4️⃣ Metrikler
    auto metrics =
        hamals_lidar_toolbox::core::ScanMetrics::compute(clean_scan, segments);

    // 5️⃣ Engel tespiti
    auto obstacle_map =
        obstacle_detector_.detect(metrics);

    // Debug output
    for (const auto& [region, state] : obstacle_map)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Region: %s | obstacle: %s | min_distance: %.2f",
            region.c_str(),
            state.has_obstacle ? "YES" : "NO",
            state.min_distance
        );
    }
}

std::vector<hamals_lidar_toolbox::core::ScanSegmenter::Region>
ScanProcessorNode::createDefaultRegions() const
{
    using Region = hamals_lidar_toolbox::core::ScanSegmenter::Region;

    // Aci araliklari (radyan)
    return {
        {"front", -0.52,  0.52},   // ~ -30° .. 30°
        {"left",   0.52,  2.09},   // ~ 30° .. 120°
        {"right", -2.09, -0.52},   // ~ -120° .. -30°
        {"rear",   2.09,  3.14}    // ~ 120° .. 180°
    };
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ScanProcessorNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}