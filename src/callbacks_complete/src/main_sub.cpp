#include "ros/ros.h"
#include "callbacks_complete/Subscriber.h"

#include <dynamic_reconfigure/server.h>
#include <callbacks_complete/ApproximationsConfig.h>
#include <callbacks_complete/WheelsConfig.h>

void approximationCallback(callbacks_complete::ApproximationsConfig &config, Subscriber* sub) {
    sub->approximationCallback(config.approximation);
}

void wheelCallback(callbacks_complete::WheelsConfig &config, Subscriber* sub, uint32_t level){
    sub->wheelParametersCallback(config.r, config.l, config.w, config.N, level);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "callbacks_sub");
  
  Subscriber my_subscriber;

  /* Momentaneamente il primo dynamic_reconfigure è commentato siccome non se ne possono fare due contemporaneamente a
   * quanto pare.
   * Per il momento quindi lo utilizziamo solo per il tuning, poi tanto non ci serverà piu e potremo toglierlo.*/

  /*dynamic_reconfigure::Server<callbacks_complete::ApproximationsConfig> server;
  dynamic_reconfigure::Server<callbacks_complete::ApproximationsConfig>::CallbackType f;
  f = boost::bind(&approximationCallback, _1, &my_subscriber);
  server.setCallback(f);*/
  dynamic_reconfigure::Server<callbacks_complete::WheelsConfig> server2;
  dynamic_reconfigure::Server<callbacks_complete::WheelsConfig>::CallbackType f2;
  f2 = boost::bind(&wheelCallback, _1, &my_subscriber, _2);
  server2.setCallback(f2);

  //rosrun rqt_reconfigure rqt_reconfigure (roscore attivo !!) è il comando per aprire l'interfaccia per modificare i parametri

  my_subscriber.main_loop();

  return 0;
}
