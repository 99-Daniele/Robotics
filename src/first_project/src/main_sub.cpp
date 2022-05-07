#include "ros/ros.h"
#include "first_project/Subscriber.h"

#include <dynamic_reconfigure/server.h>
#include <first_project/ApproximationsConfig.h>
#include <first_project/WheelsConfig.h>

void approximationCallback(first_project::ApproximationsConfig &config, Subscriber* sub) {
    sub->approximationChange(config.approximation);
}

void wheelCallback(first_project::WheelsConfig &config, Subscriber* sub, uint32_t level){
    sub->wheelParametersChange(config.r, config.l, config.w, config.N, level);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "callbacks_sub");
  
  Subscriber my_subscriber;

  /* This following lines are for create a dynamic_reconfigure server for calibration of wheel parameters
   *
   * */

  dynamic_reconfigure::Server<first_project::ApproximationsConfig> server;
  dynamic_reconfigure::Server<first_project::ApproximationsConfig>::CallbackType f;
  f = boost::bind(&approximationCallback, _1, &my_subscriber);
  server.setCallback(f);
  /*dynamic_reconfigure::Server<first_project::WheelsConfig> server2;
  dynamic_reconfigure::Server<first_project::WheelsConfig>::CallbackType f2;
  f2 = boost::bind(&wheelCallback, _1, &my_subscriber, _2);
  server2.setCallback(f2);*/

  my_subscriber.main_loop();

  return 0;
}
