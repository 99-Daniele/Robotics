#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    ROS_INFO("Linear velocity: x = %f, y = %f, z = %f", msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    ROS_INFO("Angular velocity: x = %f, y = %f, z = %f", msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub_chatter = n.subscribe("cmd_vel", 1000, velocityCallback);
    ROS_INFO("prova se funziona il nodo");
    ros::spin();

    return 0;
}
