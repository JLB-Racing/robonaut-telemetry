#include <cstdio>
#include <memory>

#include "JLB/udp.hxx"
#include "JLB/common.hxx"
#include "JLB/signals.hxx"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace jlb
{
  class Telemetry : public rclcpp::Node
  {
  public:
    Telemetry() : Node("telemetry"), server(SERVER_ADDRESS, SERVER_PORT)
    {
      RCLCPP_INFO(this->get_logger(), "node started.");

      for (const auto &signal : jlb::SignalLibrary::get_instance().signals)
      {
        publishers.push_back(this->create_publisher<std_msgs::msg::Float32>(signal.name, 10));
      }

      while (rclcpp::ok())
      {
        char msg[5] = {0};

        if (server.recv(msg, sizeof(msg)) > 0)
        {
          jlb::Value value{msg, sizeof(msg)};
          auto msg = std_msgs::msg::Float32();
          msg.data = value.value;
          publishers.at(value.id)->publish(msg);
        }
        rclcpp::spin_some(this->get_node_base_interface());
      }
    }
    ~Telemetry()
    {
      RCLCPP_INFO(this->get_logger(), "node stopped.");
    }

  private:
    UDPServer server;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers;
  };
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jlb::Telemetry>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
