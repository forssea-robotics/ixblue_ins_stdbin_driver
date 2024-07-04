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
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Received twist message");
    ixblue_stdbin_decoder::Data::BinaryNav binaryNav;

    binaryNav.dvlGroundSpeed2->dvl_id = 2;
    binaryNav.dvlGroundSpeed2->validityTime_100us = msg->header.stamp.sec * 10000 + msg->header.stamp.nanosec / 100000;
    binaryNav.dvlGroundSpeed2->dvl_speedofsound_ms = 1500.;
    binaryNav.dvlGroundSpeed2->xv1_groundspeed_ms = msg->twist.twist.linear.x;
    binaryNav.dvlGroundSpeed2->xv2_groundspeed_ms = msg->twist.twist.linear.y;
    binaryNav.dvlGroundSpeed2->xv3_groundspeed_ms = msg->twist.twist.linear.z;
    binaryNav.dvlGroundSpeed2->xv1_stddev_ms = msg->twist.covariance[0];
    binaryNav.dvlGroundSpeed2->xv2_stddev_ms = msg->twist.covariance[7];
    binaryNav.dvlGroundSpeed2->xv3_stddev_ms = msg->twist.covariance[14];

    // --- Send data
    publisher_->sendNextData(binaryNav, msg->header.stamp.sec * 10000 + msg->header.stamp.nanosec / 100000);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Sent message over udp");
}
