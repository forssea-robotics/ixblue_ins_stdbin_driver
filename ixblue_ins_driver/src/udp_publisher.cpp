#include <ixblue_ins_driver/udp_publisher.h>
#include <boost/bind.hpp>

#include <rclcpp/logging.hpp>

using namespace boost::asio;

UDPPublisher::UDPPublisher(const std::string & ip,
                         uint16_t port)
    : IPPublisher(ip, port),
      socket(service, ip::udp::endpoint(ip::address::from_string(ip), port))
{
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("udp_publisher"), "Starting asio thread");
    asioThread = std::thread([&]() { service.run(); });
}

void UDPPublisher::sendNextData(const ixblue_stdbin_decoder::Data::BinaryNav& binaryNav, uint32_t time_100us)
{
    std::vector<uint8_t> data;
    try {
        data = encoder.serialize(binaryNav, time_100us);
    } catch(std::runtime_error& e){
        // Serialization errors are reported by throwing std::runtime_exception.
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("udp_publisher"), "Serialization error : " << e.what());
    }
    std::copy(data.begin(), data.end(), datas.begin());
    socket.async_send_to(boost::asio::buffer(datas), endpoint,
                          boost::bind(&IPPublisher::handler, this,
                                      boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred));
}
