#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H


#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include <dynamic_reconfigure/server.h>
#include <callbacks_complete/ApproximationsConfig.h>
#include <callbacks_complete/WheelsConfig.h>

class Subscriber {
public:

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
  void approximationCallback(callbacks_complete::ApproximationsConfig &config);
  void wheelParametersCallback(callbacks_complete::WheelsConfig &config);
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

  /*Il server pubblica le modifiche dei parametri. A quanto pare solo un server alla volta può funzionare, quindi solo
  un file alla volta. Per il momento ho lasciato WheelsConfig che serve per modificare i parametri delle ruote per
  calcolarli. Nel momento in cui li abbiamo trovati, faremo ApproximationConfig (ora commentato) che serve invece per
   modificare il parametro sul tipo di approssimazione Eulero o Runge-Kutta.*/

  //dynamic_reconfigure::Server<callbacks_complete::ApproximationsConfig> server;
  dynamic_reconfigure::Server<callbacks_complete::WheelsConfig> server;
};

#endif