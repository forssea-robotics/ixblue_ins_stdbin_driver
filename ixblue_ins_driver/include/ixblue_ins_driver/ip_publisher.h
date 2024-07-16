#pragma once

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/noncopyable.hpp>
#include <boost/system/error_code.hpp>
#include <ixblue_stdbin_decoder/stdbin_encoder.h>
#include <string>
#include <thread>

/*!
 * \brief Contains the common part of TCP and UDP Publisher.
 * This class manage the io_service thread, and the onNewDataReceived method.
 */

class IPPublisher : private boost::noncopyable
{
    IPPublisher() = delete;

public:
    IPPublisher(std::string  ip,
               uint16_t port);
    virtual ~IPPublisher();

    virtual void sendNextData(const ixblue_stdbin_decoder::Data::BinaryNav& binaryNav, uint32_t time_100us) = 0;

    void handler(
      const boost::system::error_code& error, // Result of operation.
      std::size_t bytes_transferred           // Number of bytes sent.
    );

protected:

    const std::string ip;
    const uint16_t port;

    ixblue_stdbin_decoder::StdBinEncoder encoder;

    boost::asio::io_service service;
    std::thread asioThread;
};
