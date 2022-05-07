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
#include "callbacks_complete/setPos.h"

class Subscriber {
public:

  Subscriber();

  void main_loop();
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void wheelCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void velocityPublisher(float vx, float vy, float W, ros::Time stamp);
  void odometryPublisher(float x, float vx, float y, float vy, float theta, float W, ros::Time stamp);
  void odometryBroadcast(float x, float y, float theta, ros::Time stamp);
  void approximationCallback(int approximation);
  void wheelParametersCallback(float r, float l, float w, int N, int level);
  void setPosition(float x, float y, float theta);
  bool setServicePosition(callbacks_complete::setPos::Request  &req, callbacks_complete::setPos::Response &res);
  void changeR(float differential_new);
  void changeL(float differential_new);
  void changeW(float differential_new);
  void changeN(float differential_new);

private:

  ros::NodeHandle n;

  ros::Subscriber sub_wheel;
  ros::Subscriber sub_pose;

  ros::Publisher velocity_publisher;
  ros::Publisher odometry_publisher;
  ros::Publisher tick_vel_pub;
  ros::ServiceServer service;

  tf2_ros::TransformBroadcaster br;

  float r;
  float r_avg = 0.07;
  float l;
  float l_avg = 0.2;
  float w;
  float w_avg = 0.169;
  int N;
  float N_avg = 42.0;
  int count = 0;
  int T;
  int approximationType;
  bool poseSetted = false;
  float x_old = 0.0, y_old = 0.0, theta_old = 0.0;
  float differential_old = 0.0;
  float differential_avg = 0.0;
  bool positive = true, positiveN = true;
  float old_ticks[4]= {0.0,0.0,0.0,0.0};
  ros::Time old_time;
};

#endif