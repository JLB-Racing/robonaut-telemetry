#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <memory>

class Telemetry : public rclcpp::Node
{
public:
  Telemetry() : Node("telemetry")
  {
    RCLCPP_INFO(this->get_logger(), "Hello, world!");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Telemetry>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
