#include "odom_updater.h"

int main(int argc, char * argv[]){
    
    //init
    rclcpp::init(argc, argv);   //initialize the communication with ROS 

    //node
    auto node = std::make_shared<Broadcaster>("odom_updater");     // creating a node "hello" with shared pointer from Hello class
    rclcpp::spin(node);

    //shutdown
    rclcpp::shutdown();
}