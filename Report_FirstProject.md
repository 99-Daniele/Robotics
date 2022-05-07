# Robotics - first project

DA TOGLIERE BAG DAL LAUNCH

- 10649263 Lorenzo Nebolosi
- 10630561 Daniele Cicala
- 10620037 Elena Rosato


- ***ROS FILES:***

  - **launch.launch** 

    is the launch file. It starts all the nodes and the world_pose static TF transformation to align frames.
    There is also the list of all ROS parameters.

  - **main_sub.cpp** 

    used for the dynamic reconfigure ----!!NON NE SONO SICURA!!!!(o da spostare?)

  - **Subscriber.cpp** implements the node sub_wheel, which have different functions: 
    - *wheelCallbacks* 
      
      reads the ticks from wheel_states and computes the RPM and publish it in the topic *ticks_to_RPM* (we used it to check if our computation was correct). 
  
        We used the formula:
        
          RPM = (current_ticks - past_ticks) * 2 * pi / (N * T * deltaTime) 
            
        and than saved the current ticks as the past ones. 
        Then it computes the robot speed:
              
          Vx = (v_fl + v_fr + v_rl + v_rr) * r/4
          Vy = (-v_fl + v_fr + v_rl - v_rr) * r/4
          W = (-v_fl + v_fr - v_rl + v_rr) * r/(4 * (l + w))
  
        and use the function *velocityPublisher* to publish them in the topic cdm_vel.
        Next step is the integration, the method is chosen via dynamic reconfigure (by default is Euler method).
        We call *odometryPublisher* and *odometryBroadcast*, functions than respectively publish the odometry (in "odom") and broadcast it.
        At the end the current bag time and space parameters are saved to be used in the next callback as the old ones.
         DA DESCRIVERE:
         ODOMETRY AND TF!!!
    - *setServicePosition*
            
       set new current position based on request.
        
    - *setInitialPosition* 
        
       set initial position based on initial values directly taken from bags.

    - *approximationChange*
    
      change current approximation from Euler to Runge-Kutta and viceversa after dynamic_reconfigure request.
       
    - *wheelParamtersChange*
      
      change wheel parameters after dynamic_reconfigure request.
        
    - *setPosition*
        
      set new current position

  - **velocity.cpp** 
  
    it's of the class sub_pub, used to read cdm_vel, compute the wheel speeds and publish it in wheels_rpm as a custom message RPM.
    
    We used the following formulas:
   
        w_fl=(vx-vy-(l+w)*wz)/r;
        w_fr=(vx+vy+(l+w)*wz)/r;
        w_rl=(vx+vy-(l+w)*wz)/r;
        w_rr=(vx-vy+(l+w)*wz)/r;

    We checked with plotjuggler that it was all matching.

  - **"ApproximationParameters.cfg"**

    is a dynamic reconfigure file which change the current approximation from Euler to Runge-Kutta and viceversa


  - **"WheelParameters.cfg"**

    is a dynamic reconfigure file which change wheel parameters. Those are wheel radius, wheel positions along x.
    wheel position along y and CPR. This file was created for calibration of these parameters.


  - **RPM.msg**

    this is the required custom message. Its parameters are one Header and 4 Float referring to wheels velocity
    expressed in RPM.


  - **setPos.srv**

    mettere descrizione....
    rosservice call /setPos 0 0 0


  - **"SubPub.h"** 

    ->spiego template!!! e il resto PER nodo CHE FACCIA DA SUB E DA PUB


  - **Subscriber.h** 

    It's a class that contains all the needed parameters
    spiego a cosa servono gli old!!!!!


- ***ROS PARAMETERS:***
  - *r*: wheel radius. Different from the one given because of calibration.
  - *l*: wheel position along x. Different from the one given because of calibration.
  - *w*: wheel position along y. Different from the one given because of calibration.
  - *N*: encoders resolution. Different from the one given because of calibration.
  - *T*: gear ration
  - *initialApproximation*: default value for approximation. In this case is 0 which refers to Euler approximation.
  - *x*: initial x position of robot. Directly taken from each bags.
  - *y*: initial y position of robot. Directly taken from each bags.
  - *z*: initial z position of robot. Directly taken from each bags.
  - *qx*: initial x orientation of robot. Directly taken from each bags.
  - *qy*: initial y orientation of robot. Directly taken from each bags.
  - *qz*: initial z orientation of robot. Directly taken from each bags.
  - *qw*: initial w orientation of robot. Directly taken from each bags.

- ***RQT_GRAPH STRUCTURES***

  mettere png
  
- ***TF TREE STRUCTURES:***

  mettere png


- ***CUSTOM MESSAGES:***

  Our unique custom message is RPM.msg which is the one requested.


- ***START/USE NODES:***

  roslaunch launch.launch


- ***CALIBRATION:*** 

  To calibrate given parameters we use dynamic reconfigure to change r, l, w and N.
  We firstly concentrate on bag1 which only perform translations so only r and N parameters are used in formulas. 
  After that, we moved to bag2 and bag3 to calibrate l and w to obtain the better approximation of given position.
  We mainly used rviz and plotjuggler to confront our odometry with the given one on robot/pose.

- ***RESULT CHECKING***

  ticks topic aggiunto da noi...

