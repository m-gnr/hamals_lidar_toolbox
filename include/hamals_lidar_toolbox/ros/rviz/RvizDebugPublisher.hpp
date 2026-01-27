#pragma once

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace hamals_lidar_toolbox::ros::rviz
{

class RvizDebugPublisher
{
public:
  explicit RvizDebugPublisher(rclcpp::Node& node);

  // boş publish (test)
  void publishEmpty();

  // FAZ 4.2 – front fan çizimi
  void publishFrontFan(
      double angle_min,
      double angle_max,
      double radius,
      bool has_obstacle
  );

private:
  rclcpp::Publisher<
      visualization_msgs::msg::MarkerArray
  >::SharedPtr pub_;
};

}  // namespace hamals_lidar_toolbox::ros::rviz
