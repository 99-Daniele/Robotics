#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <sstream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "callbacks_pub");
  ros::NodeHandle n;

  ros::Publisher counter_pub = n.advertise<std_msgs::Int32>("count", 1000);

  int count = 0;

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    // generate counter msg
    std_msgs::Int32 count_msg;
    count_msg.data = count;

    // print count to screen
    // ROS_INFO("%d", count);

    // publish messages
    counter_pub.publish(count_msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }

  return 0;
}
