#include <ixblue_ins_driver/udp_listener.h>
#include <boost/bind.hpp>

#include <rclcpp/logging.hpp>

using namespace boost::asio;

UDPListener::UDPListener(const std::string & ip,
                         uint16_t port,
                         StdBinDataHandlerInterface * data_handler)
    : IPListener(ip, port, data_handler),
      socket(service, ip::udp::endpoint(ip::address::from_string(ip), port))
{
    listenNextData();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("udp_listener"), "Starting asio thread");
    asioThread = std::thread([&]() { service.run(); });
}

void UDPListener::listenNextData()
{
    socket.async_receive_from(boost::asio::buffer(datas), endpoint,
                              boost::bind(&IPListener::onNewDataReceived, this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));
}
