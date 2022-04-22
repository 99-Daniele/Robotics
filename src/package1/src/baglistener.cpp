//
// Created by Lorenzo Nebolosi on 22/04/22.
//
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    return 0;
}
