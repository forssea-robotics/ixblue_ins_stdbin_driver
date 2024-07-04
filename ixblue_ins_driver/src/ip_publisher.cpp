#include <ixblue_ins_driver/ip_publisher.h>

#include <ixblue_stdbin_decoder/stdbin_encoder.h>
#include <utility>
#include <rclcpp/logging.hpp>

using namespace boost::asio;

IPPublisher::IPPublisher(std::string ip,
                       uint16_t port)
  : ip(std::move(ip)), port(port), encoder(ixblue_stdbin_decoder::StdBinEncoder::ProtocolVersion::V3, ixblue_stdbin_decoder::StdBinEncoder::DataMode::INPUT)
{}

IPPublisher::~IPPublisher()
{
    service.stop();
    asioThread.join();
}

void IPPublisher::handler(
      const boost::system::error_code& error, // Result of operation.
      std::size_t bytes_transferred           // Number of bytes sent.
   )
{
    if (error)
    {
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger("ip_publisher"), "Error occurs in IP Publisher : " << error.message());
    }
    else {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ip_publisher"), "Data sent: " << bytes_transferred << " bytes");
    }
}
