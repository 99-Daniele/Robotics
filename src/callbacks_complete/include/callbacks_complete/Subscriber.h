#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H


#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"


class Subscriber {
public:
  float r=0.07;
  float l=0.200;
  float w=0.169;
  float T=5;
  int N=42;


  Subscriber(); 
  void main_loop();

//void countCallback(const std_msgs::Int32::ConstPtr& msg);
//  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void wheelCallback(const sensor_msgs::JointState::ConstPtr& msg);

private:
  ros::NodeHandle n; 
  ros::Subscriber sub_wheel;
  //ros::Subscriber sub_pose;

  ros::Publisher velocity_publisher;
  ros::Publisher odometry_publisher;

   float x_old=0,y_old=0,theta_old=0;
   float old_ticks[4]= {0,0,0,0};
   ros::Time old_time;
};

#endif