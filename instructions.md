# Robotics - first project

DA TOGLIERE BAG DAL LAUNCH
parametri da launch
####Students:
- 10649263 Lorenzo Nebolosi
- Daniele Cicala
- 10620037 Elena Rosato
##Description of the source files:

- **launch.launch** is the launch file. It starts all the nodes and the initial pose for the odometry
?---anche i parametri---?


- **RPM.msg** the custom message (as shown in the professor slide)


- **main_sub.cpp** used for the dynamic reconfigure ----!!NON NE SONO SICURA!!!!(o da spostare?)


- **Subscriber.cpp** implements the node sub_wheel, which have different functions: 
  - *wheelCallbacks* reads the ticks from wheel_states and computes the RPM and publish it in the topic *ticks_to_RPM* (we used it to check if our computation was correct). 
  
    We used the formula: RPM=(current_ticks-past_ticks)*2*pi/(N*T*deltaTime)
      and than saved the current ticks as the past ones.
    > Then it computes the robot speed:
    Vx=(v_fl+v_fr+v_rl+v_rr)*r/4
    Vy=(-v_fl+v_fr+v_rl-v_rr)*r/4
    W=(-v_fl+v_fr-v_rl+v_rr)*r/(4*(l+w))
  
  - And use the function *velocityPublisher* to pubblish them in the topic cdm_vel.
  - Next step is the integration, the method is choosen via dynamic reconfigure(by default is Euler method).
  - We call *odometryPublisher* and *odometryBroadcast*, functions than respectively pubblish the odometry(in "odom") and broadcast it.
  - At the end the current bag time and space paramethers are saved to be used in the next callback as the old ones.
   DA DESCRIVERE:
   ODOMETRY AND TF!!!


- We then have a service called by setPose which uses the setServicePosition function which expect three inputs values representing the x, y and theta (in radiants) of the new position
     ``` 
     rosservice call /setPos 0 0 0
     ```
   change parameters!!!
   approxCallback


- *velocity.cpp* it's of the class sub_pub, used to read cdm_vel, compute the wheel speeds and publish it in wheels_rpm as a custom message RPM.

##Other informations about the project:
- We used the following formulas:
 > w_fl=(vx-vy-(l+w)*wz)/r;

> w_fr=(vx+vy+(l+w)*wz)/r;

> w_rl=(vx+vy-(l+w)*wz)/r;

> w_rr=(vx-vy+(l+w)*wz)/r;

We chacked with plotjuggler that it was all matching.


- "ApproximationParameters.cfg" !!!!!!!!!!!!
- "WheelParameters.cfg"!!!!!!!!!!!!!!!

- "SubPub.h" ->spiego template!!! e il resto PER nodo CHE FACCIA DA SUB E DA PUB

- **Subscriber.h** It's a class that contains all the needed parameters
spiego a cosa servono gli old!!!!!

- name and meaning of the ROS parameters //DA FARE
- structure of the TF tree
- As custom message we only used RPM, as requested in the slide
- description of how to start/use the nodes
- info you think are important/interesting

##Calibration
We calibrated the parameters with dynamic reconfigure, starting from bag one, we defined a proper r and N that would make our motion coincide with the robot in linear movement. Than we focused on the other bag to adjust the other paramethers regarding the rotational motion.




