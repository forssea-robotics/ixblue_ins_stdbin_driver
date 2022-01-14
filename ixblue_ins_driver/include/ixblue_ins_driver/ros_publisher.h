#pragma once

#include <ixblue_ins_msgs/msg/ins.hpp>
#include <ixblue_stdbin_decoder/data_models/nav_header.h>
#include <ixblue_stdbin_decoder/data_models/stdbin.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include "diagnostics_publisher.h"
#include "std_bin_data_handler_interface.hpp"

class ROSPublisher : public StdBinDataHandlerInterface
{
public:
    explicit ROSPublisher(rclcpp::Node::SharedPtr node);
    void onNewStdBinData(const ixblue_stdbin_decoder::Data::BinaryNav& navData,
                         const ixblue_stdbin_decoder::Data::NavHeader& headerData) override;

    // Standard ros msgs
    static bool toImuMsg(const ixblue_stdbin_decoder::Data::BinaryNav & navData,
                         bool use_compensated_acceleration,
                         sensor_msgs::msg::Imu & imu_msg);

    static bool toNavSatFixMsg(const ixblue_stdbin_decoder::Data::BinaryNav & navData,
                               sensor_msgs::msg::NavSatFix & navsat_msg);

    bool toTimeReference(const ixblue_stdbin_decoder::Data::NavHeader & headerData,
                                sensor_msgs::msg::TimeReference & time_ref_msg);

    // iXblue ros msgs
    static bool toiXInsMsg(const ixblue_stdbin_decoder::Data::BinaryNav & navData,
                    ixblue_ins_msgs::msg::Ins & ix_ins_msg);

protected:
    // Header
    bool getHeader(const ixblue_stdbin_decoder::Data::NavHeader & headerData,
                   const ixblue_stdbin_decoder::Data::BinaryNav & navData,
                   std_msgs::msg::Header & header_msg);

    // Launch parameters
    std::string frame_id;
    std::string time_source;
    std::string time_origin;
    bool use_compensated_acceleration;

    rclcpp::Node::SharedPtr node_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr stdImuPublisher;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr stdNavSatFixPublisher;
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr stdTimeReferencePublisher;
    rclcpp::Publisher<ixblue_ins_msgs::msg::Ins>::SharedPtr stdInsPublisher;
    DiagnosticsPublisher diagPub;

    // Utils
    bool useInsAsTimeReference = true;
    bool useUnixAsTimeOrigin = true;
};
