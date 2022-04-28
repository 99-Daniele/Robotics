#include "ros/ros.h"
//#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    ROS_INFO("POSITION: x = %f, y = %f, z = %f", msg->pose.pose.position.x, msg->pose.pose.position.y,  msg->pose.pose.position.z);
    ROS_INFO("ORIENTATION: x = %f, y = %f, z = %f, w = %f", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("LINEAR VELOCITY: x = %f, y = %f, z = %f", msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    ROS_INFO("ANGULAR VELOCITY: x = %f, y = %f, z = %f", msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber od_sub = n.subscribe("odom", 1000, odometryCallback);
    ros::spin();

    return 0;
}
