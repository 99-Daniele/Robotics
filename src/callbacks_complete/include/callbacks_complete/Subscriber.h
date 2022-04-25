#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H


#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

class Subscriber {
public:
  Subscriber(); 
  void main_loop();

//void countCallback(const std_msgs::Int32::ConstPtr& msg);
//  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void wheelCallback(const sensor_msgs::JointState::ConstPtr& msg);

private:
  ros::NodeHandle n; 
  ros::Subscriber sub_wheel;
  //ros::Subscriber sub_pose;

  ros::Publisher pub;
 // int sum;

 float old_ticks[4]= {0,0,0,0};
 ros::Time old_time;
};

#endif