//
// Created by Lorenzo Nebolosi on 22/04/22.
//
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO("My pose_position: %f, %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("My pose_orientation: %f, %f, %f, %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

void wheelCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for(int i = 0; i < msg->position.size(); i++) {
        ROS_INFO("Name - %d: %s", i, msg->name[i].c_str());
        ROS_INFO("Position - %d: %f", i, msg->position[i]);
        ROS_INFO("Velocity - %d: %f", i, msg->velocity[i]);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub_pose = n.subscribe("robot/pose", 1000, poseCallback);
    ros::Subscriber sub_wheel = n.subscribe("wheel_states", 1000, wheelCallback);

    ros::spin();

    return 0;
}