#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "callbacks_complete/Subscriber.h"
#include "callbacks_complete/RPM.h"
#include "callbacks_complete/SubPub.h"

template<>
void SubPub<geometry_msgs::TwistStamped,callbacks_complete::RPM>::subscriberCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    float vx = msg->twist.linear.x;
    float vy = msg->twist.linear.y;
    float vz = msg->twist.linear.z;

    float wx = msg->twist.angular.x;
    float wy = msg->twist.angular.y;
    float wz = msg->twist.angular.z;

    ROS_INFO("Linear velocity: vx = %f, vy = %f, vz = %f", vx, vy, vz);
    ROS_INFO("Angular velocity: wx = %f, wy = %f, wz = %f", wx, wy, wz);

    float r,l,w;
    n.getParam("/r", r);
    n.getParam("/l", l);
    n.getParam("/w", w);

    callbacks_complete::RPM wheel_speed;

    wheel_speed.header.stamp = msg->header.stamp;

    wheel_speed.rpm_fl=(vx-vy-(l+w)*wz)/r;
    wheel_speed.rpm_fr=(vx+vy+(l+w)*wz)/r;
    wheel_speed.rpm_rl=(vx+vy-(l+w)*wz)/r;
    wheel_speed.rpm_rr=(vx-vy+(l+w)*wz)/r;

    ROS_INFO("fl: %f, fr: %f, rl: %f, rr: %f", wheel_speed.rpm_fl,wheel_speed.rpm_fr,wheel_speed.rpm_rl,wheel_speed.rpm_rr);

    publisherObject.publish(wheel_speed);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "velocity");

    SubPub<geometry_msgs::TwistStamped,callbacks_complete::RPM>velocity("cmd_vel","wheels_rpm",1000);

    ros::spin();

    return 0;
}