#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

// timer
class TargetReacher : public rclcpp::Node
{
public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {
        // params 
        this->declare_parameter<double>("aruco_target.x");
        this->declare_parameter<double>("aruco_target.y");
        this->declare_parameter<std::string>("final_destination.frame_id");
        this->declare_parameter<double>("final_destination.aruco_0.x");
        this->declare_parameter<double>("final_destination.aruco_0.y");
        this->declare_parameter<double>("final_destination.aruco_1.x");
        this->declare_parameter<double>("final_destination.aruco_1.y");
        this->declare_parameter<double>("final_destination.aruco_2.x");
        this->declare_parameter<double>("final_destination.aruco_2.y");
        this->declare_parameter<double>("final_destination.aruco_3.x");
        this->declare_parameter<double>("final_destination.aruco_3.y");
        m_aruco_target_x = this->get_parameter("aruco_target.x").as_double();
        m_aruco_target_y = this->get_parameter("aruco_target.y").as_double();
        m_final_destination_frame_id = this->get_parameter("final_destination.frame_id").as_string();
        m_final_destination_aruco_0_x = this->get_parameter("final_destination.aruco_0.x").as_double();
        m_final_destination_aruco_0_y = this->get_parameter("final_destination.aruco_0.y").as_double();
        m_final_destination_aruco_1_x = this->get_parameter("final_destination.aruco_1.x").as_double();
        m_final_destination_aruco_1_y = this->get_parameter("final_destination.aruco_1.y").as_double();
        m_final_destination_aruco_2_x = this->get_parameter("final_destination.aruco_2.x").as_double();
        m_final_destination_aruco_2_y = this->get_parameter("final_destination.aruco_2.y").as_double();
        m_final_destination_aruco_3_x = this->get_parameter("final_destination.aruco_3.x").as_double();
        m_final_destination_aruco_3_y = this->get_parameter("final_destination.aruco_3.y").as_double();
        
        // Static Broadcaster
        tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // this->broadcaster_callback(goal);

        // // Listener
        
        // t = tf_buffer->lookupTransform("final_destination", "robot1/odom", tf2::TimePointZero); 

        // Subscribers
        m_subscriber = this->create_subscription<std_msgs::msg::Bool>("goal_reached", 10, std::bind(&TargetReacher::goal1_callback,this, std::placeholders::_1));
        m_aruco_subscriber = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10, std::bind(&TargetReacher::aruco_callback, this, std::placeholders::_1));

        m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("robot1/cmd_vel", 10);

        m_bot_controller = bot_controller;
        m_bot_controller->set_goal(m_aruco_target_x, m_aruco_target_y);
        
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
    }

private:
    // attributes
    std::shared_ptr<BotController> m_bot_controller;
    geometry_msgs::msg::Twist m_msg;

    bool rotate{true};

    // subscriber
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscriber;
    void goal1_callback(const std_msgs::msg::Bool::SharedPtr msg);
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr m_aruco_subscriber;
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
    // publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;

    // Static Broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
    void broadcaster_callback(const std::array<double,2> goal);

    // Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    geometry_msgs::msg::TransformStamped t;
    // void Listner(const tf2_ros::TransformListener tf_listener, geometry_msgs::msg::TransformStamped t);
    
    // params
    double m_aruco_target_x;
    double m_aruco_target_y;
    std::string m_final_destination_frame_id;
    double m_final_destination_aruco_0_x;
    double m_final_destination_aruco_0_y;
    double m_final_destination_aruco_1_x;
    double m_final_destination_aruco_1_y;
    double m_final_destination_aruco_2_x;
    double m_final_destination_aruco_2_y;
    double m_final_destination_aruco_3_x;
    double m_final_destination_aruco_3_y;
    
    std::array<double,2> goal{};
    // void final_destination(const geometry_msgs::msg::TransformStamped t);

    
};