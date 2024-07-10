#include <ixblue_ins_driver/udp_publisher.h>
#include <ixblue_ins_driver/ros_subscriber.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr n = std::make_shared<rclcpp::Node>("ixblue_ins_input_driver");

  std::string ip;
  int udp_port;
  n->declare_parameter<int>("udp_port", 8200);
  n->declare_parameter<std::string>("ip", std::string("0.0.0.0"));
  n->get_parameter("udp_port", udp_port);
  n->get_parameter("ip", ip);

  RCLCPP_INFO(n->get_logger(), "UDP port : %d", udp_port);
  RCLCPP_INFO(n->get_logger(), "IP address : %s", ip.c_str());

  if (udp_port > std::numeric_limits<uint16_t>::max()) {
    RCLCPP_ERROR_STREAM(
      n->get_logger(),
      "UDP Port can't be greater than : " << std::numeric_limits<uint16_t>::max());
    return -1;
  }

  auto udp_publisher = std::make_unique<UDPPublisher>(ip, static_cast<uint16_t>(udp_port));

  ROSSubscriber ros_subscriber(n, udp_publisher.get());


  rclcpp::spin(n);
  rclcpp::shutdown();
  return 0;
}