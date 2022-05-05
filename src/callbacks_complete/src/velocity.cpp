#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "callbacks_complete/Subscriber.h"
#include "callbacks_complete/RPM.h"
#include "SubPub.h"

template<>
void SubPub<geometry_msgs::TwistStamped,callbacks_complete::RPM>::subscriberCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    ROS_INFO("Linear velocity: x = %f, y = %f, z = %f", msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    ROS_INFO("Angular velocity: x = %f, y = %f, z = %f", msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
    //float w0,w1,w2,w3;
    float r, l, w;
    n.getParam("/r", r);
    n.getParam("/l", l);
    n.getParam("/w", w);
/*
    ros::NodeHandle n;

    ros:: Publisher wheels_rpm_pub = n.advertise<callbacks_complete::RPM>("wheels_rpm", 1000);
*/
    callbacks_complete::RPM wheel_speed;
    wheel_speed.header.stamp = msg->header.stamp;
    wheel_speed.rpm_fl=1/r*(msg->twist.linear.x-msg->twist.linear.y-(l+w)*msg->twist.angular.z);
    wheel_speed.rpm_fr=1/r*(msg->twist.linear.x+msg->twist.linear.y+(l+w)*msg->twist.angular.z);
    wheel_speed.rpm_rl=1/r*(msg->twist.linear.x+msg->twist.linear.y-(l+w)*msg->twist.angular.z);
    wheel_speed.rpm_rr=1/r*(msg->twist.linear.x-msg->twist.linear.y+(l+w)*msg->twist.angular.z);

    ROS_INFO("fl: %f, fr: %f, rl: %f, rr: %f", wheel_speed.rpm_fl,wheel_speed.rpm_fr,wheel_speed.rpm_rl,wheel_speed.rpm_rr);
    publisherObject.publish(wheel_speed);


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity");

    SubPub<geometry_msgs::TwistStamped,callbacks_complete::RPM>velocity("cmd_vel","wheels_rpm",1000);
   /* ros::NodeHandle n;

    ros::Subscriber sub_chatter = n.subscribe("cmd_vel", 1000, velocityCallback);
    ROS_INFO("prova se funziona il nodo");

    //ros:: Publisher wheels_rpm_pub = n.advertise<callbacks_complete::RPM>("wheels_rpm", 1000);
*/
    ros::spin();

    return 0;
}