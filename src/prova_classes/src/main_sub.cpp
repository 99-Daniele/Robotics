#include "ros/ros.h"
#include "callbacks_complete/Subscriber.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "callbacks_sub");
  
  Subscriber my_subscriber;

  my_subscriber.main_loop();

  return 0;
}
