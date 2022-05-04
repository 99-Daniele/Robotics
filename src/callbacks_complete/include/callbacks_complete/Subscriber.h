#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H


#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <callbacks_complete/ApproximationsConfig.h>
#include <callbacks_complete/WheelsConfig.h>

class Subscriber {
public:

  tf2_ros::TransformBroadcaster br;

  float r=0.07;
  float l=0.200;
  float w=0.169;
  int N=42;
  float T=5;

  int approximationType = 0;

  Subscriber(); 
  void main_loop();

//void countCallback(const std_msgs::Int32::ConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void wheelCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void velocityPublisher(float vx, float vy, float W, ros::Time stamp);
  void odometryPublisher(float x, float vx, float y, float vy, float theta, float W, ros::Time stamp);
  void odometryBroadcast(float x, float vx, float y, float vy, float theta, float W, ros::Time stamp);
  void approximationCallback(int approximation);
  void wheelParametersCallback(float r, float l, float w, int N, int level);
  void setPosition(float, float, float);

private:
  ros::NodeHandle n;
  ros::Subscriber sub_wheel;
  ros::Subscriber sub_pose;

  ros::Publisher velocity_publisher;
  ros::Publisher odometry_publisher;

  bool poseSetted = false;
  float x_old = 0, y_old = 0, theta_old = 0;
  float old_ticks[4]= {0,0,0,0};
  ros::Time old_time;
};

#endif