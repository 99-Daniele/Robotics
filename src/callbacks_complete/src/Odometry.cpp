#include "ros/ros.h"
//#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    ROS_INFO("Position: x= %f,y = %f, z = %f", msg->pose.pose.position.x, msg->pose.pose.position.y,  msg->pose.pose.position.z);
    ROS_INFO("Linear velocity: x = %f, y = %f, z = %f", msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    ROS_INFO("Angular velocity: x = %f, y = %f, z = %f", msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber od_sub = n.subscribe("odom", 1000, odometryCallback);
    ROS_INFO("prova se funziona il nodo");
    ros::spin();

    return 0;
}
