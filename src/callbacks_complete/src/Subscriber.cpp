#define _USE_MATH_DEFINES
 
#include <cmath>
#include <iostream>
#include "callbacks_complete/Subscriber.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

Subscriber::Subscriber() { // class constructor
  // all initializations here
//  this->sub = this->n.subscribe("count", 1000, &Subscriber::countCallback, this);
  this->sub_wheel = this->n.subscribe("wheel_states", 1000, &Subscriber::wheelCallback, this);

  this->velocity_publisher = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  this->odometry_publisher = this->n.advertise<nav_msgs::Odometry>("odom", 1000);

//  this->old_ticks; //l'ho inizializzato in subscriber.h ma non son sicura che sia corretto
  this->old_time=ros::Time::now();
}

void Subscriber::main_loop() {
  ros::Rate loop_rate(10);

  while (ros::ok()) {

    ros::spinOnce();

    loop_rate.sleep();
  }
}

/*
void Subscriber::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO("My pose_position: %f, %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("My pose_orientation: %f, %f, %f, %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}
*/
void Subscriber::wheelCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    /*PRINTING BAG VALUES
     *
     * for(int i = 0; i < msg->position.size(); i++) {
        ROS_INFO("Name - %d: %s", i, msg->name[i].c_str());
        ROS_INFO("Position - %d: %f", i, msg->position[i]);
        ROS_INFO("Velocity - %d: %f", i, msg->velocity[i]);


    }
     */
    //FROM TICKS TO robot velocity
    float vx;//robot velocity along x
    float vy;//robot speed along y
    float W;//robot angular velocity

    float numVx;//sono step intermedi perchè una volta diviso per il tempo potrebbe dare problemi se i tempi sono infinitesimali
    float numVy;
    float numW;

    numVx = (msg->position[0] - this->old_ticks[0] + msg->position[ 1] - this->old_ticks[1] + msg->position[2] - this->old_ticks[2] + msg->position[3] - this->old_ticks[3]);
    numVy = (-(msg->position[0] - this->old_ticks[0]) + msg->position[1] - this->old_ticks[1] + msg->position[2] - this->old_ticks[2] - (msg->position[3] - this->old_ticks[3]));
    numW = (-(msg->position[0] - this->old_ticks[0]) + msg->position[1] - this->old_ticks[1] - (msg->position[2] - this->old_ticks[2]) + (msg->position[3] - this->old_ticks[3]));

    //ROS_INFO("Duration seconds: %d, and nanoseconds: %d", msg->header.stamp.sec, msg->header.stamp.nsec);
    //ROS_INFO("Duration in seconds: %lf", msg->header.stamp.toSec());
        //vx=numVx*2*pi/((time-past_time)*N(=42)*T(=5))*r/4
    vx = numVx*r*M_PI / N / 2 / T / (msg->header.stamp - this->old_time).toSec();
    vy = numVy*r*M_PI / N / 2 / T / (msg->header.stamp - this->old_time).toSec();
    W = numVy*r*M_PI / N / 2 / T /(l+w)/ (msg->header.stamp - this->old_time).toSec();



    for (int i = 0; i < msg->position.size(); i++){
        this->old_ticks[i]=msg->position[i];
    }

    ROS_INFO("numVx: %f, vx: %f,   numVy %f, vy: %f", numVx, vx, numVy,vy);


    //codice per pubblicare v e w. considero vx = vx, vy = vy, vz = 0; wx = 0, wy = 0, wz = W (non sono sicura che l'angular sia solo wz, ma direi di si)

    //l'ho commentato perchè faCCIO L'ADVERTISE UNA VOLTA SOLA, altrimenti mi spesso non vedevo il topic
    // ros::Publisher velocity_publisher = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

    geometry_msgs::TwistStamped velocity_msg;

    /* non so se serve mettere anche l'header, ho trovato sta formula ma non credo sia giusta
    Header header = std_msgs.msg.Header();
    header.stamp = rospy.Time.now();
    velocity_msg.header = header;*/
    velocity_msg.header.stamp = msg->header.stamp;//per sincronizzare i dati inviati con il tempo del bag(msg->header.stamp)

    velocity_msg.twist.linear.x = vx;
    velocity_msg.twist.linear.y = vy;
    velocity_msg.twist.linear.z = 0;
    velocity_msg.twist.angular.x = 0;
    velocity_msg.twist.angular.y = 0;
    velocity_msg.twist.angular.z = W;

    this->velocity_publisher.publish(velocity_msg);

    //**INTEGRATION OF vx, vy and W to find the pose (x,y,theta)
    float x,y,theta,Ts;

    Ts=(msg->header.stamp - this->old_time).toSec();
    //Euler method
    x=this->x_old + vx*Ts*cos(this->theta_old)-vy*Ts*sin(this->theta_old);
    y=this->y_old + vx*Ts*sin(this->theta_old)+vy*Ts*cos(this->theta_old);
    theta=this->theta_old + W*Ts;
//sarà da implementare runge-kutta(e poi vanno scelti con un if con dymanic reconfigure)


  //odometry pubblisher
    nav_msgs::Odometry odometry_msg;

    /* non so se serve mettere anche l'header, ho trovato sta formula ma non credo sia giusta
    Header header = std_msgs.msg.Header();
    header.stamp = rospy.Time.now();
    odometry_msg.header = header;*/
    odometry_msg.header.stamp = msg->header.stamp;//per sincronizzare i dati inviati con il tempo del bag(msg->header.stamp)
    odometry_msg.header.frame_id = "odom";

    //set the position
    odometry_msg.pose.pose.position.x = x;
    odometry_msg.pose.pose.position.y = y;
    odometry_msg.pose.pose.position.z = 0.0;
  //  odometry_msg.pose.pose.orientation = odom_quat;//devo crearla!!!! con tf2
    //set the velocity
    odometry_msg.child_frame_id = "base_link";
    odometry_msg.twist.twist.linear.x = vx;
    odometry_msg.twist.twist.linear.y = vy;
    odometry_msg.twist.twist.angular.z = W;

    //publish the message
    this->odometry_publisher.publish(odometry_msg);


    this->old_time=msg->header.stamp;


    ros::spinOnce();

}
