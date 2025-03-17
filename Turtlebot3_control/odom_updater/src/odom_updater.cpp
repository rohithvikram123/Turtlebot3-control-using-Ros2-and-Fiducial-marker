#include <rclcpp/rclcpp.hpp>
#include "odom_updater.h"

void Broadcaster::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "robot1/odom";
    t.child_frame_id = "robot1/base_footprint";

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    tf_broadcaster->sendTransform(t);
}