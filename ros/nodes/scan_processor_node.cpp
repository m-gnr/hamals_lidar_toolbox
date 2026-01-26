#include "scan_processor_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "hamals_lidar_msgs/msg/obstacle_state.hpp"
#include "hamals_lidar_msgs/msg/obstacle_region_state.hpp"

#include "hamals_lidar_toolbox/ros/adapters/LaserScanAdapter.hpp"
#include "hamals_lidar_toolbox/core/ScanMetrics.hpp"

using std::placeholders::_1;

ScanProcessorNode::ScanProcessorNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("scan_processor_node", options),
  sanitizer_(0.05, 30.0),                 // min_range, max_range
  segmenter_(createDefaultRegions())      // region definitions
{
    // Danger distance (LiDAR-only, sabit)
    const double danger_distance = 0.5;
    obstacle_detector_.setDangerDistance(danger_distance);

    // /scan subscriber
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&ScanProcessorNode::scanCallback, this, _1)
    );

    // /scan/obstacle_state publisher
    obstacle_state_pub_ =
        this->create_publisher<hamals_lidar_msgs::msg::ObstacleState>(
            "/scan/obstacle_state",
            10
        );

    RCLCPP_INFO(this->get_logger(), "ScanProcessorNode started.");
}

void ScanProcessorNode::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 1️⃣ ROS -> Core
    hamals_lidar_toolbox::core::ScanData scan =
        hamals_lidar_toolbox::ros::adapters::LaserScanAdapter::fromRosMessage(*msg);

    // 2️⃣ Sanitize
    hamals_lidar_toolbox::core::ScanData clean_scan =
        sanitizer_.sanitize(scan);

    // 3️⃣ Segment
    auto segments = segmenter_.segment(clean_scan);

    // 4️⃣ Metrics
    auto metrics =
        hamals_lidar_toolbox::core::ScanMetrics::compute(clean_scan, segments);

    // 5️⃣ Obstacle detection
    auto obstacle_map =
        obstacle_detector_.detect(metrics);

    // 6️⃣ Core -> ROS msg
    hamals_lidar_msgs::msg::ObstacleState msg_out;

    for (const auto& [region, state] : obstacle_map)
    {
        hamals_lidar_msgs::msg::ObstacleRegionState r;
        r.region = region;
        r.has_obstacle = state.has_obstacle;
        r.min_distance = state.min_distance;

        msg_out.regions.push_back(r);
    }

    // 7️⃣ Publish
    obstacle_state_pub_->publish(msg_out);

   /* // Debug log (LiDAR-only, kalabilir)
    for (const auto& [region, state] : obstacle_map)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Region: %s | obstacle: %s | min_distance: %.2f",
            region.c_str(),
            state.has_obstacle ? "YES" : "NO",
            state.min_distance
        );
    }*/
}

std::vector<hamals_lidar_toolbox::core::ScanSegmenter::Region>
ScanProcessorNode::createDefaultRegions() const
{
    using Region = hamals_lidar_toolbox::core::ScanSegmenter::Region;

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
