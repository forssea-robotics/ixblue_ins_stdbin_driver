#pragma once

#include "ip_publisher.h"
#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <ixblue_stdbin_decoder/data_models/stdbin.h>

class TCPClientPublisher : public IPPublisher
{
    TCPClientPublisher() = delete;
public:
    TCPClientPublisher(std::string ip, uint16_t port);

    void sendNextData(const ixblue_stdbin_decoder::Data::BinaryNav& binaryNav, uint32_t time_100us);

protected:
    boost::asio::ip::tcp::socket socket;
    boost::asio::ip::tcp::resolver resolver;
};