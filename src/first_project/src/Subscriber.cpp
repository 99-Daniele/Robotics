#define _USE_MATH_DEFINES
 
#include <cmath>
#include <iostream>
#include "first_project/Subscriber.h"
#include "first_project/RPM.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "first_project/setPos.h"

Subscriber::Subscriber() { // class constructor
  // all initializations here
  this->sub_wheel = this->n.subscribe("wheel_states", 1000, &Subscriber::wheelCallback, this);
  this->sub_pose = this->n.subscribe("/robot/pose", 1000, &Subscriber::poseCallback, this);//SERVE QUESTO?

  this->velocity_publisher = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  this->odometry_publisher = this->n.advertise<nav_msgs::Odometry>("odom", 1000);
  this->tick_vel_pub = this->n.advertise<first_project::RPM>("ticks_to_RPM",100);
  this->service = n.advertiseService("setPos", &Subscriber::setServicePosition, this);

  this->old_time=ros::Time::now();
}

void Subscriber::main_loop() {
  ros::Rate loop_rate(10);
  n.getParam("/r", r);
  n.getParam("/l", l);
  n.getParam("/w", w);
  n.getParam("/N", N);
  n.getParam("/T", T);
  n.getParam("/initialApproximation", approximationType);
  while (ros::ok()) {

    ros::spinOnce();

    loop_rate.sleep();
  }
}

bool Subscriber::setServicePosition(first_project::setPos::Request  &req, first_project::setPos::Response &res)
{
    setPosition(req.x, req.y, req.theta);

    //this->x_old = req.x;
    //this->y_old = req.y;
    //this->theta_old = req.theta;
    ROS_INFO("Read value: x = %f, y = %f and theta = %f", req.x, req.y, req.theta);
    ROS_INFO("Updated the robot position to: x = %f, y = %f and theta = %f", this->x_old, this->y_old, this->theta_old);
    return true;
}

void Subscriber::wheelCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    
    //FROM TICKS TO robot velocity
    float vx;//robot velocity along x
    float vy;//robot speed along y
    float W;//robot angular velocity

    float Ts;
    Ts = (msg->header.stamp - this->old_time).toSec(); // delta time

    //ticks to RPM ci serve per poi confrontarla con gli RPM calcolati in velocity
    first_project::RPM ticks_RPM;
    ticks_RPM.header.stamp = msg->header.stamp;
    ticks_RPM.rpm_fl=((msg->position[0] - this->old_ticks[0])*2*M_PI)/(N * T * Ts);
    ticks_RPM.rpm_fr=((msg->position[1] - this->old_ticks[1])*2*M_PI)/(N * T *Ts);
    ticks_RPM.rpm_rl=((msg->position[2] - this->old_ticks[2])*2*M_PI)/(N * T *Ts);
    ticks_RPM.rpm_rr=((msg->position[3] - this->old_ticks[3])*2*M_PI)/(N * T *Ts);

    this->tick_vel_pub.publish(ticks_RPM);

   /* //calcolo lungo TUTTO QUESTO CREDO CHE SI POSSA TOGLIERE
    float numVx;//sono step intermedi perchè una volta diviso per il tempo potrebbe dare problemi se i tempi sono infinitesimali
    float numVy;
    float numW;

    numVx = ((msg->position[0] - this->old_ticks[0]) + (msg->position[1] - this->old_ticks[1]) + (msg->position[2] - this->old_ticks[2]) + (msg->position[3] - this->old_ticks[3]));
    numVy = (-(msg->position[0] - this->old_ticks[0]) + (msg->position[1] - this->old_ticks[1]) + (msg->position[2] - this->old_ticks[2]) - (msg->position[3] - this->old_ticks[3]));
    numW = (-(msg->position[0] - this->old_ticks[0]) + (msg->position[1] - this->old_ticks[1]) - (msg->position[2] - this->old_ticks[2]) + (msg->position[3] - this->old_ticks[3]));

    //ROS_INFO("Duration seconds: %d, and nanoseconds: %d", msg->header.stamp.sec, msg->header.stamp.nsec);
    //ROS_INFO("Duration in seconds: %lf", msg->header.stamp.toSec());
        //vx=numVx*2*pi/((time-past_time)*N(=42)*T(=5))*r/4
    vx = (numVx*r*M_PI) / (N * 2 * T * (msg->header.stamp - this->old_time).toSec());
    vy = (numVy*r*M_PI) / (N * 2 * T * (msg->header.stamp - this->old_time).toSec());
    W = (numW*r*M_PI) / (N * 2 * T * (l + w) * (msg->header.stamp - this->old_time).toSec());
*/

//from RPM to robot velocity
    vx=(ticks_RPM.rpm_fl+ticks_RPM.rpm_fr+ticks_RPM.rpm_rl+ticks_RPM.rpm_rr)*r/4;
    vy=(-ticks_RPM.rpm_fl+ticks_RPM.rpm_fr+ticks_RPM.rpm_rl-ticks_RPM.rpm_rr)*r/4;
    W=(-ticks_RPM.rpm_fl+ticks_RPM.rpm_fr-ticks_RPM.rpm_rl+ticks_RPM.rpm_rr)*r/(4*(l+w));


    for (int i = 0; i < msg->position.size(); i++){
        this->old_ticks[i]=msg->position[i];
    };

    velocityPublisher(vx, vy, W, msg->header.stamp);


    //**INTEGRATION OF vx, vy and W to find the pose (x,y,theta)

    float x,y,theta;

    if(this->approximationType == 0) {
        //Euler method
        x = this->x_old + vx * Ts * cos(this->theta_old) - vy * Ts * sin(this->theta_old);
        y = this->y_old + vx * Ts * sin(this->theta_old) + vy * Ts * cos(this->theta_old);
        theta = this->theta_old + W * Ts;
    }
    else {
        //Runge Kutta
        x = this->x_old + vx * Ts * cos(this->theta_old + (W * Ts / 2)) - vy * Ts * sin(this->theta_old + (W * Ts / 2));
        y = this->y_old + vx * Ts * sin(this->theta_old + (W * Ts / 2)) + vy * Ts * cos(this->theta_old + (W * Ts / 2));
        theta = this->theta_old + W * Ts;
    }

    odometryPublisher(x, vx, y, vy, theta, W, msg->header.stamp);
    odometryBroadcast(x, y, theta, msg->header.stamp);

    this->old_time=msg->header.stamp;
    this->x_old = x;
    this->y_old = y;
    this->theta_old = theta;

    ros::spinOnce();
}

void Subscriber::velocityPublisher(float vx, float vy, float W, ros::Time stamp){

    geometry_msgs::TwistStamped velocity_msg;

    velocity_msg.header.stamp = stamp;

    velocity_msg.twist.linear.x = vx;
    velocity_msg.twist.linear.y = vy;
    velocity_msg.twist.linear.z = 0;

    velocity_msg.twist.angular.x = 0;
    velocity_msg.twist.angular.y = 0;
    velocity_msg.twist.angular.z = W;

    this->velocity_publisher.publish(velocity_msg);
}

void Subscriber::odometryPublisher(float x, float vx, float y, float vy, float theta, float W, ros::Time stamp){

    nav_msgs::Odometry odometry_msg;

    odometry_msg.header.stamp = stamp;
    odometry_msg.header.frame_id = "odom";
    odometry_msg.child_frame_id = "base_link";

    //set the position
    odometry_msg.pose.pose.position.x = x;
    odometry_msg.pose.pose.position.y = y;
    odometry_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    odometry_msg.pose.pose.orientation.x = q.x();
    odometry_msg.pose.pose.orientation.y = q.y();
    odometry_msg.pose.pose.orientation.z = q.z();
    odometry_msg.pose.pose.orientation.w = q.w();

    //set the velocity
    odometry_msg.twist.twist.linear.x = vx;
    odometry_msg.twist.twist.linear.y = vy;
    odometry_msg.twist.twist.linear.z = 0.0;

    odometry_msg.twist.twist.angular.x = 0.0;
    odometry_msg.twist.twist.angular.y = 0.0;
    odometry_msg.twist.twist.angular.z = W;

    //publish the message
    this->odometry_publisher.publish(odometry_msg);
}

void Subscriber::odometryBroadcast(float x, float y, float theta, ros::Time stamp){

    geometry_msgs::TransformStamped odometry_tf;

    odometry_tf.header.stamp = stamp;
    odometry_tf.header.frame_id = "odom";
    odometry_tf.child_frame_id = "base_link";

    odometry_tf.transform.translation.x = x;
    odometry_tf.transform.translation.y = y;
    odometry_tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    odometry_tf.transform.rotation.x = q.x();
    odometry_tf.transform.rotation.y = q.y();
    odometry_tf.transform.rotation.z = q.z();
    odometry_tf.transform.rotation.w = q.w();

    br.sendTransform(odometry_tf);
}

void Subscriber::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) { //da prendere val nel launch
    if(!poseSetted){
        tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        this->setPosition(msg->pose.position.x, msg->pose.position.y, (float)yaw);

        ROS_INFO("POSE SETTED");
        ROS_INFO("My pose_position: %f, %f, %f, %f, %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, yaw, pitch, roll);
        poseSetted = true;
    }
    /*float pose_x = msg->pose.position.x;
    float odom_x = x_old;
    float pose_y = msg->pose.position.y;
    float odom_y = y_old;
    float differential = (odom_x - pose_x) * (odom_y - pose_y);
    changeR(differential);
    changeL(differential);
    changeW(differential);
    changeN(differential);
    float total_r = r_avg * count;
    float total_l = l_avg * count;
    float total_w = w_avg * count;
    float total_N = N_avg * count;
    float total_d = differential_avg * count;
    count++;
    r_avg = (total_r + r)/count;
    l_avg = (total_l + l)/count;
    w_avg = (total_w + w)/count;
    N_avg = (total_N + N)/count;
    differential_avg = (total_d + differential)/count;
    ROS_INFO("old_diff: %f  new:diff: %f  diff_avg: %f  r_avg: %f  l_avg: %f  w_avg: %f  N_avg: %f count: %d", differential_old, differential, differential_avg, r_avg, l_avg, w_avg, N_avg, count);
    differential_old = differential;*/
}

void Subscriber::changeR(float differential_new){
    if(differential_old < - 0.0005) {
        if (differential_new < differential_old) {
            if(positive)
                r = r - 0.0001;
            else
                r = r + 0.0001;
            positive = !positive;
        }
        else{
            if(positive)
                r = r + 0.0001;
            else
                r = r - 0.0001;
        }
    }
    else if(differential_old > 0.0005){
        if (differential_new < differential_old) {
            if(positive)
                r = r - 0.0001;
            else
                r = r + 0.0001;
            positive = !positive;
        }
        else{
            if(positive)
                r = r + 0.0001;
            else
                r = r - 0.0001;
        }
    }
}

void Subscriber::changeL(float differential_new){
    if(differential_old < - 0.0005) {
        if (differential_new < differential_old) {
            if(positive)
                l = l - 0.0001;
            else
                l = l + 0.0001;
            positive = !positive;
        }
        else{
            if(positive)
                l = l + 0.0001;
            else
                l = l - 0.0001;
        }
    }
    else if(differential_old > 0.0005){
        if (differential_new < differential_old) {
            if(positive)
                l = l - 0.0001;
            else
                l = l + 0.0001;
            positive = !positive;
        }
        else{
            if(positive)
                l = l + 0.0001;
            else
                l = l - 0.0001;
        }
    }
}

void Subscriber::changeW(float differential_new){
    if(differential_old < - 0.0005) {
        if (differential_new < differential_old) {
            if(positive)
                w = w - 0.00001;
            else
                w = w + 0.00001;
            positive = !positive;
        }
        else{
            if(positive)
                w = w + 0.00001;
            else
                w = w - 0.00001;
        }
    }
    else if(differential_old > 0.0005){
        if (differential_new < differential_old) {
            if(positive)
                w = w - 0.00001;
            else
                w = w + 0.00001;
            positive = !positive;
        }
        else{
            if(positive)
                w = w + 0.00001;
            else
                w = w - 0.00001;
        }
    }
}

void Subscriber::changeN(float differential_new){
    if(differential_old < - 0.0005) {
        if (differential_new < differential_old) {
            if(positiveN)
                if(N > 37)
                    N = N - 1;
                else
                    positiveN = !positiveN;
            else
                if( N < 47)
                    N = N + 1;
                else
                    positiveN = !positiveN;
            positiveN = !positiveN;
        }
        else{
            if(positiveN)
                if(N < 47)
                    N = N + 1;
                else
                    positiveN = !positiveN;
            else
                if(N > 37)
                    N = N - 1;
                else
                    positiveN = !positiveN;
        }
    }
    else if(differential_old > 0.0005){
        if (differential_new < differential_old) {
            if(positiveN)
                if(N > 37)
                    N = N - 1;
                else
                    positiveN = !positiveN;
            else
                if(N < 47)
                    N = N + 1;
                else
                    positiveN = !positiveN;
            positiveN = !positiveN;
        }
        else{
            if(positive)
                if(N < 47)
                    N = N + 1;
                else
                    positiveN = !positiveN;
            else
                if(N > 37)
                    N = N - 1;
                else
                    positiveN = !positiveN;
        }
    }
}

void Subscriber::approximationCallback(int approximation){
    if(approximation == 0)
        ROS_INFO("Approximation changed: EULER");
    else
        ROS_INFO("Approximation changed: RUNGE-KUTTA");
    this->approximationType = approximation;
}

void Subscriber::wheelParametersCallback(float r, float l, float w, int N, int level){
    switch(level){
        case 0:
            this->r = r;
            ROS_INFO("r parameter changed: %f", this->r);
            break;
        case 1:
            this->l = l;
            ROS_INFO("l parameter changed: %f", this->l);
            break;
        case 2:
            this->w = w;
            ROS_INFO("w parameter changed: %f", this->w);
            break;
        case 3:
            this->N = N;
            ROS_INFO("N parameter changed: %d", this->N);
            break;
    }
    ROS_INFO("r: %f, l: %f, w: %f, N: %d", this->r, this->l, this->w, this->N);
}

void Subscriber::setPosition(float x, float y, float theta){
    this->x_old = x;
    this->y_old = y;
    this->theta_old = theta;
}