#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

class Broadcaster: public rclcpp::Node{
    public:
        Broadcaster(std::string odom_updater): Node(odom_updater){
            // broadcaster
            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            
            //subscriber
            m_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("robot1/odom", 10, std::bind(&Broadcaster::odom_callback, this, std::placeholders::_1));
        }

    private:
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriber;
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};  