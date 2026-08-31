#include <rclcpp/rclcpp.hpp>
#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_coverage_planner_3d_ns::SensorCoveragePlanner3D>();
  node->initialize();
  rclcpp::spin(node);
  // spin() returns once rclcpp's SIGINT/SIGTERM handler has shut the context down and the executor stopped,
  // so the final SemPathBench snapshot runs on the main thread with no callback in flight.
  node->ExportSemPathSnapshot("final");
  rclcpp::shutdown();
  return 0;
}