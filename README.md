# robotics
ROS projects for robotics class

***!!! launch senza bag!!!***


Goals

I.Compute odometry using appropriate kinematics
 - Compute robot linear and angular velocities v, ⍵ from wheel encoders with mecanum wheels kinematics
  1. Write down the formula to compute v, ⍵ from wheel speeds *
  2. Adapt formula to use encoder ticks (more precise) instead of RPM ***ci domandavamo se serve passare attraverso rpm***
  3. Compute a rough estimate of v, ⍵ (with given robot parameters fixed)
  4. Publish v, ⍵ as topic cmd_vel of type geometry_msgs/TwistStamped
 - Compute odometry from v, ⍵ using both Euler and Runge-Kutta integration 
  1. Start with Euler, add Runge-Kutta later 
  2. Add ROS parameter for initial pose (x,y,θ) ***??????????(DA GROUND TRUTH)Lorenzo***
  3. Publish as nav_msgs/Odometry on topic odom
  4. Broadcast TF odom->base_link ***lab5*** ***Daniele***
  5. - structure of the TF tree ***DA GENERARE PER IL FILE***
- Calibrate (fine-tune) robot parameters (r, l, w, N) to match ground truth ***fatto...Elena->rviz e posizione ottenuta***

II. Compute wheel control speeds from v, ⍵
 - Compute wheel speeds (RPM) from v, ⍵  
  1. Reverse the formula obtained at the previous step (I.1) 
  2. Read v, ⍵ from cmd_vel and apply the obtained formula 
  3. Publish the computed wheel speed as custom message on topic
     wheels_rpm
    -The custom message has prototype:
      Header header
      float64 rpm_fl
      float64 rpm_fr
      float64 rpm_rr
      float64 rpm_rl
 - Check that the results match the recorded encoders values, apart from some noise
     You could use rqt_plot or plotjuggler  ***Elena***
III. Add a service to reset the odometry to a specified pose (x,y,θ)  ***Lorenzo***
IV. Use dynamic reconfigure to select between integration method     ***fatto**
   - Use an enum with 2 values: Euler, RK



Omnidirectional robot
Mecanum wheels
- 4 wheels with rollers at 45°
Encoders on each wheel
- RPM (very noisy)
- Ticks (more accurate)
Geometric parameters:
- Wheel radius (r)
- Wheel position along x (±l)
- Wheel position along y (±w)

given:
- Wheel radius (r): 0.07 m (could be a bit off)
- Wheel position along x (l): 0.200 m
- Wheel position along y (w): 0.169 m
- Gear ratio (T): 5:1
- Encoders resolution (N): 42 CPR (Counts Per Rev.) (could be a bit off)
