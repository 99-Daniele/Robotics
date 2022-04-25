#define _USE_MATH_DEFINES
 
#include <cmath>
#include <iostream>
#include "callbacks_complete/Subscriber.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"

Subscriber::Subscriber() { // class constructor
  // all initializations here
//  this->sub = this->n.subscribe("count", 1000, &Subscriber::countCallback, this);
  this->sub_wheel = this->n.subscribe("wheel_states", 1000, &Subscriber::wheelCallback, this);

  this->pub = this->n.advertise<std_msgs::Int32>("sum", 1000);
  this->velocity_publisher = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);


    this->old_ticks;
  this->old_time=ros::Time::now();
 // this->sum = 0;
}

void Subscriber::main_loop() {
  ros::Rate loop_rate(10);

  while (ros::ok()) {

    //ROS_INFO("Current sum: %d", this->sum);

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
    float vx;
    float vy;
    float numVx;//sono step intermedi perchè una volta diviso per il tempo potrebbe dare problemi se i tempi sono infinitesimali
    float numVy;
    numVx = (msg->position[0] - this->old_ticks[0] + msg->position[ 1] - this->old_ticks[1] + msg->position[2] - this->old_ticks[2] + msg->position[3] - this->old_ticks[3]);
    numVy = (-(msg->position[0] - this->old_ticks[0]) + msg->position[1] - this->old_ticks[1] + msg->position[2] - this->old_ticks[2] - (msg->position[3] - this->old_ticks[3]));
    //ROS_INFO("Duration seconds: %d, and nanoseconds: %d", msg->header.stamp.sec, msg->header.stamp.nsec);
    //ROS_INFO("Duration in seconds: %lf", msg->header.stamp.toSec());
        //vx=numVx*2*pi/((time-past_time)*N(=42)*T(=5))
    vx = numVx*2*M_PI  / (msg->header.stamp - this->old_time).toSec(); //msg->Time ?? msg è di tipo sensor_msgs::JointState che non ha un campo Time
    vy = numVy*2*M_PI  / (msg->header.stamp - this->old_time).toSec(); //msg->Time ?? msg è di tipo sensor_msgs::JointState che non ha un campo Time

    for (int i = 0; i < msg->position.size(); i++){
        this->old_ticks[i]=msg->position[i];
    }
    this->old_time=msg->header.stamp;

    ROS_INFO("numVx: %f, vx: %f,   numVy %f, vy: %f", numVx, vx, numVy,vy);


    //codice per pubblicare v e w. Per il momento considero vx = vx, vy = vy, vz = 0; wx = 3, wy = 4, wz = 5 per vedere se funziona correttamente

    //l'ho commentato perchè faCCIO L'ADVERTISE UNA VOLTA SOLA, altrimenti mi spesso non vedevo il topic
    // ros::Publisher velocity_publisher = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

    geometry_msgs::TwistStamped velocity_msg;

    /* non so se serve mettere anche l'header, ho trovato sta formula ma non credo sia giusta
    Header header = std_msgs.msg.Header();
    header.stamp = rospy.Time.now();
    velocity_msg.header = header;*/
    velocity_msg.header.stamp=ros::Time::now();

    velocity_msg.twist.linear.x = vx;
    velocity_msg.twist.linear.y = vy;
    velocity_msg.twist.linear.z = 0;
    velocity_msg.twist.angular.x = 3;
    velocity_msg.twist.angular.y = 4;
    velocity_msg.twist.angular.z = 5;

    this->velocity_publisher.publish(velocity_msg);
    ros::spinOnce();

}


/*
void Subscriber::countCallback(const std_msgs::Int32::ConstPtr& msg) {
  ROS_INFO("Received: %d", msg->data);

  // sum is a member variable (i.e., it is defined inside the class)
  // --> we can access it from anywhere inside the class (using "this->")!
  this->sum = this->sum + msg->data;

  // We can use this->pub without advertising it every time
  // (advertising it every time would have been very inefficient)
  std_msgs::Int32 sum_msg;
  sum_msg.data = this->sum;
  this->pub.publish(sum_msg);
}
*/