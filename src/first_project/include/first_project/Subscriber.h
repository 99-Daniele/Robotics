#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/ApproximationsConfig.h>
#include <first_project/WheelsConfig.h>
#include "first_project/setPos.h"

class Subscriber {
public:

  Subscriber();

  void main_loop();
  void wheelCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void approximationChange(int approximation);
  void wheelParametersChange(float r, float l, float w, int N, int level);

private:

  ros::NodeHandle n;

  ros::Subscriber sub_wheel;

  ros::Publisher velocity_publisher;
  ros::Publisher odometry_publisher;
  ros::Publisher tick_vel_publisher;

  ros::ServiceServer service;

  tf2_ros::TransformBroadcaster br;

  float r, l, w;
  int N, T;
  int approximationType;
  bool start = false;
  float x_old = 0.0, y_old = 0.0, theta_old = 0.0;
  float old_ticks[4]= {0.0,0.0,0.0,0.0};
  ros::Time old_time;

  void velocityPublisher(float vx, float vy, float W, ros::Time stamp);
  void odometryPublisher(float x, float vx, float y, float vy, float theta, float W, ros::Time stamp);
  void odometryBroadcast(float x, float y, float theta, ros::Time stamp);
  void setInitialPosition();
  void setPosition(float x, float y, float theta);
  bool setServicePosition(first_project::setPos::Request  &req, first_project::setPos::Response &res);
};

#endif