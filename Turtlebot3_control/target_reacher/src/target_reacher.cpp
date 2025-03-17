#include <rclcpp/rclcpp.hpp>
#include "target_reacher.h"

void TargetReacher::goal1_callback(const std_msgs::msg::Bool::SharedPtr msg){
    if(rotate){
        std::cout << msg << std::boolalpha << "\n";
        m_msg.angular.z = 0.2;
        RCLCPP_INFO(this->get_logger(), "Publishing angular_velocity for the robot to rotate");
        m_publisher->publish(m_msg);
    }
}

void TargetReacher::aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){
    if(rotate){
        int i = msg->marker_ids[0];
        RCLCPP_INFO_STREAM(this->get_logger(), "Marker id of the detected aruco_marker: " << i);

        switch(i){
            case 0:
                goal = {m_final_destination_aruco_0_x, m_final_destination_aruco_0_y};
                break;
            case 1:
                goal = {m_final_destination_aruco_1_x, m_final_destination_aruco_1_y};
                break;
            case 2:
                goal = {m_final_destination_aruco_2_x, m_final_destination_aruco_2_y};
                break;
            case 3:
                goal = {m_final_destination_aruco_3_x, m_final_destination_aruco_3_y};
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Incorrect aruco_marker_id");
                break;     
        }
        RCLCPP_INFO(this->get_logger(), "Goal coordinate: {%f, %f}", goal[0], goal[1]);
        m_msg.angular.z = 0.0;
        m_publisher->publish(m_msg);    
        rotate = 0;
    }

    broadcaster_callback(goal);
    
     
}

void TargetReacher::broadcaster_callback(const std::array<double,2> goal){
    
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = m_final_destination_frame_id;
    t.child_frame_id = "final_destination";

    t.transform.translation.x = goal[0];
    t.transform.translation.y = goal[1];
    t.transform.translation.z = 0;

    t.transform.rotation.x = 0;
    t.transform.rotation.y = 0;
    t.transform.rotation.z = 0;
    t.transform.rotation.w = 1;

    tf_static_broadcaster->sendTransform(t);

    try {
        geometry_msgs::msg::TransformStamped transformStamped = tf_buffer->lookupTransform("robot1/odom", "final_destination", tf2::TimePointZero);
        
        m_bot_controller->set_goal(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
                
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }

    



}

// void TargetReacher::Listner(tf2_ros::TransformListener tf_listener, geometry_msgs::msg::TransformStamped t){
//     tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
//     t = tf_buffer->lookupTransform("final_destination", "robot1/odom", tf2::TimePointZero);
//     // m_bot_controller->set_goal(t.transform.translation.x, t.transform.translation.y);
// }