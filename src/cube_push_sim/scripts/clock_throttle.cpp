// Limit simulation clock fan-out; keep Gazebo time and backward resets intact.
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("simulation_clock_throttle");
  auto pub = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  bool initialized = false;
  int64_t last = 0;
  auto sub = node->create_subscription<rosgraph_msgs::msg::Clock>(
    "/simulation/clock_raw", rclcpp::ClockQoS(),
    [&](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
      const int64_t now = static_cast<int64_t>(msg->clock.sec) * 1000000000LL + msg->clock.nanosec;
      if (!initialized || now < last || now - last >= 20000000LL) {
        pub->publish(*msg);
        last = now;
        initialized = true;
      }
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
