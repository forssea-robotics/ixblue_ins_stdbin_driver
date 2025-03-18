#pragma once

#include "ip_publisher.h"
#include <boost/asio.hpp>
#include <ixblue_stdbin_decoder/data_models/stdbin.h>

class UDPPublisher : public IPPublisher
{
    UDPPublisher() = delete;
public:
    UDPPublisher(const std::string & ip,
                uint16_t port);

    void sendNextData(const ixblue_stdbin_decoder::Data::BinaryNav& binaryNav, uint32_t time_100us);

protected:
    boost::asio::ip::udp::socket socket;
    boost::asio::ip::udp::endpoint endpoint;
};
