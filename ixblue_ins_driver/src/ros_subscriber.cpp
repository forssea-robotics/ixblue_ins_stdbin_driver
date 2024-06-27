#include <bitset>
#include <cmath>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ixblue_ins_driver/ros_subscriber.h>


ROSSubscriber::ROSSubscriber(rclcpp::Node::SharedPtr node, IPPublisher* publisher) : node_(node), publisher_(publisher)
{
    // Subscriber
    twistSubscriber = node_->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "twist", 1, std::bind(&ROSSubscriber::onNewTwistReceived, this, std::placeholders::_1));
}

void ROSSubscriber::onNewTwistReceived(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    ixblue_stdbin_decoder::Data::BinaryNav binaryNav;
    ixblue_stdbin_decoder::Data::NavHeader headerData;

    // --- Header
    headerData.navigationDataValidityTime_100us = msg->header.stamp.sec * 10000 +
                                                  msg->header.stamp.nanosec / 100000;

    // --- Send data
    publisher_->sendNextData(binaryNav, headerData.navigationDataValidityTime_100us);
}
