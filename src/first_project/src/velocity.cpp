#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "first_project/Subscriber.h"
#include "first_project/RPM.h"
#include "first_project/SubPub.h"

template<>
void SubPub<geometry_msgs::TwistStamped,first_project::RPM>::subscriberCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    float vx = msg->twist.linear.x;
    float vy = msg->twist.linear.y;
    float vz = msg->twist.linear.z;

    float wx = msg->twist.angular.x;
    float wy = msg->twist.angular.y;
    float wz = msg->twist.angular.z;

    float r,l,w;
    n.getParam("/r", r);
    n.getParam("/l", l);
    n.getParam("/w", w);

    first_project::RPM wheel_speed;

    wheel_speed.header.stamp = msg->header.stamp;

    wheel_speed.rpm_fl=(vx-vy-(l+w)*wz)/r;
    wheel_speed.rpm_fr=(vx+vy+(l+w)*wz)/r;
    wheel_speed.rpm_rl=(vx+vy-(l+w)*wz)/r;
    wheel_speed.rpm_rr=(vx-vy+(l+w)*wz)/r;

    publisherObject.publish(wheel_speed);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "velocity");

    SubPub<geometry_msgs::TwistStamped,first_project::RPM>velocity("cmd_vel","wheels_rpm",1000);

    ros::spin();

    return 0;
}