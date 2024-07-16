#include <boost/asio/connect.hpp>
#include <ixblue_ins_driver/tcp_client_publisher.h>
#include <boost/bind.hpp>

#include <rclcpp/logging.hpp>

using namespace boost::asio;

TCPClientPublisher::TCPClientPublisher(std::string ip, uint16_t port)
    : IPPublisher(ip, port),
      socket(service),
      resolver(service)
{
    try {
        connect(socket, resolver.resolve({ip, std::to_string(port)}));
    } catch (boost::system::system_error& e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("publisher"), "Error while connecting to " << ip << ":" << port << " : " << e.what());
    }
}

void TCPClientPublisher::sendNextData(const ixblue_stdbin_decoder::Data::BinaryNav& binaryNav, uint32_t time_100us)
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
