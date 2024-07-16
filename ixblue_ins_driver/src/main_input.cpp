#include <ixblue_ins_driver/udp_publisher.h>
#include <ixblue_ins_driver/tcp_client_publisher.h>
#include <ixblue_ins_driver/ros_subscriber.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr n = std::make_shared<rclcpp::Node>("ixblue_ins_input_driver");

  std::string ip;
  int port;
  std::string connection_type;
  n->declare_parameter<int>("port", 8200);
  n->declare_parameter<std::string>("ip", std::string("0.0.0.0"));
  n->declare_parameter<std::string>("connection_type", std::string("udp"));
  n->get_parameter("port", port);
  n->get_parameter("ip", ip);
  n->get_parameter("connection_type", connection_type);

  RCLCPP_INFO(n->get_logger(), "Port : %d", port);
  RCLCPP_INFO(n->get_logger(), "IP address : %s", ip.c_str());
  RCLCPP_INFO(n->get_logger(), "Connection type : %s", connection_type.c_str());

  if (port > std::numeric_limits<uint16_t>::max()) {
    RCLCPP_ERROR_STREAM(
      n->get_logger(),
      "Port can't be greater than : " << std::numeric_limits<uint16_t>::max());
    return -1;
  }

  if (connection_type == "udp") {
    auto udp_publisher = std::make_unique<UDPPublisher>(ip, static_cast<uint16_t>(port));
    ROSSubscriber ros_subscriber(n, udp_publisher.get());
    rclcpp::spin(n);
  } else if (connection_type == "tcp_client") {
    auto tcp_client_publisher = std::make_unique<TCPClientPublisher>(ip, static_cast<uint16_t>(port));
    ROSSubscriber ros_subscriber(n, tcp_client_publisher.get());
    rclcpp::spin(n);
  } else {
    RCLCPP_ERROR_STREAM(n->get_logger(), "Unknown connection type : " << connection_type);
    return -1;
  }
  rclcpp::shutdown();
  return 0;
}
