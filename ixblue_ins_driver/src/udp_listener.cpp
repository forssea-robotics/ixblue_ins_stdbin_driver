#include "udp_listener.h"
#include <boost/bind.hpp>

using namespace boost::asio;

UDPListener::UDPListener(const std::string& ip, uint16_t port, const rclcpp::Node::SharedPtr& node)
    : IPListener(ip, port, node),
      socket(service, ip::udp::endpoint(ip::address::from_string(ip), port))
{
    listenNextData();
    RCLCPP_DEBUG_STREAM(node->get_logger(), "Starting asio thread");
    asioThread = std::thread([&]() { service.run(); });
}

void UDPListener::listenNextData()
{
    socket.async_receive_from(boost::asio::buffer(datas), endpoint,
                              boost::bind(&IPListener::onNewDataReceived, this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));
}
