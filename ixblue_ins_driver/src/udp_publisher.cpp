#include <ixblue_ins_driver/udp_publisher.h>
#include <boost/bind.hpp>

#include <rclcpp/logging.hpp>

using namespace boost::asio;

UDPPublisher::UDPPublisher(const std::string & ip,
                         uint16_t port)
    : IPPublisher(ip, port),
      endpoint(ip::address::from_string(ip), port),
      socket(service)
{
    socket.open(ip::udp::v4());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("publisher"), "Starting asio thread");
    asioThread = std::thread([&]() { service.run(); });
}

void UDPPublisher::sendNextData(const ixblue_stdbin_decoder::Data::BinaryNav& binaryNav, uint32_t time_100us)
{
    std::vector<uint8_t> data;
    try {
        RCLCPP_DEBUG(rclcpp::get_logger("publisher"), "Serializing data");
        data = encoder.serialize(binaryNav, time_100us);
        // print data in hexa
        std::stringstream ss;
        for (const auto& d : data) {
            ss << std::hex << static_cast<int>(d) << " ";
        }
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("publisher"), "Data to send : " << ss.str());
        socket.async_send(boost::asio::buffer(data),
                            boost::bind(&IPPublisher::handler, this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
    } catch(std::runtime_error& e){
        // Serialization errors are reported by throwing std::runtime_exception.
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("publisher"), "Serialization error : " << e.what());
    }
}
