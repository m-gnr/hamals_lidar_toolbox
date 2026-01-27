#pragma once

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace hamals_lidar_toolbox::ros::rviz
{

class RvizDebugPublisher
{
public:
  explicit RvizDebugPublisher(rclcpp::Node& node);

  /// Tüm marker’ları temizlemek için boş publish
  void publishEmpty();

  /// Tek bir region için fan (wrap-aware)
  void publishRegionFan(
      const std::string& region_name,
      double angle_min,
      double angle_max,
      double radius,
      bool has_obstacle);

private:
  /// Tek bir fan marker üretir (publish etmez)
  visualization_msgs::msg::Marker createFanMarker(
      const std::string& ns,
      int id,
      double angle_min,
      double angle_max,
      double radius,
      bool has_obstacle);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

}  // namespace hamals_lidar_toolbox::ros::rviz
