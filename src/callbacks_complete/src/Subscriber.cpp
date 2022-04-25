#include "callbacks_complete/Subscriber.h"

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

Subscriber::Subscriber() { // class constructor
  // all initializations here
//  this->sub = this->n.subscribe("count", 1000, &Subscriber::countCallback, this);
  this->sub_wheel = this->n.subscribe("wheel_states", 1000, &Subscriber::wheelCallback, this);

  this->pub = this->n.advertise<std_msgs::Int32>("sum", 1000);
  this->sum = 0;
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
    for(int i = 0; i < msg->position.size(); i++) {
        ROS_INFO("Name - %d: %s", i, msg->name[i].c_str());
        ROS_INFO("Position - %d: %f", i, msg->position[i]);
        ROS_INFO("Velocity - %d: %f", i, msg->velocity[i]);
        //ROS_INFO("Effort - %d: %f", i, msg->effort[i]); commentato dato che non so perché dà segmentation fault <=perchè il campo è vuoto
    }
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