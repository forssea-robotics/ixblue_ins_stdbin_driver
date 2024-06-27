#pragma once

#include <geometry_msgs/msg/detail/twist_with_covariance_stamped__struct.hpp>
#include <ixblue_stdbin_decoder/data_models/nav_header.h>
#include <ixblue_stdbin_decoder/data_models/stdbin.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/subscription.hpp>

#include "ip_publisher.h"

class ROSSubscriber
{
public:
    explicit ROSSubscriber(rclcpp::Node::SharedPtr node, IPPublisher* publisher);
    
    void onNewTwistReceived(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

protected:
    rclcpp::Node::SharedPtr node_;

    IPPublisher* publisher_;

    // Publishers
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twistSubscriber;
};
