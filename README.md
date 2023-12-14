# Repo overview
This repo contains a RSI interface and a simulation environment for the KUKA KR16 industrial robot. It was developed for YLab internal use.
The repo allows for simulation in gazebo using either trajectory or postion control. 


For installation follow the guide in: [INSTALLTION.md](https://github.com/user/repo/blob/branch/other_file.m)
## Relevant launch files
Some helpful launch files to get you started!
### Gazebo demo
To launch a simple demo of the gazebo simulation run:

 roslaunch kuka_kr16_gazebo demo_gazebo.launch

### GMO demo
To run the a gmo first launch the Gazebo demo then run:

roslaunch kuka_gmo kuka_gmo.launch


The using RVIZ generate and execute a plan into the boxes placed by the robot. Upon collision the robot should stop moving and the momentum observer will throw a collision error.

### Postion control demo
To run the postion control demo run:

 roslaunch kuka_position_control kuka_kr16_position_controller.launch




In a seperate terminal run:


rostopic pub /target_pose geometry_msgs/TransformStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
child_frame_id: ''
transform:
  translation:
    x: 1.0
    y: 0.0
    z: 0.5
  rotation:
    x: 0.0
    y: 0.707
    z: 0.0
    w: 0.707"

## Trajectory controller
The trajectory controller is a MoveIt! melodic implementation that is setup to work both as standard or as simulated in Gazebo. The trajectory controller has serveral planners aviable thanks to MoveIt! and is STRICTLY an oflline controller.


## Postion controller
The postion controller is a custom, bare bones example of a postion controller. It uses the URDF model to construct a KDL representation of the robot to calculate the inverse kinematics. It functions a a simple Servo Position server.
The position controller uses joint limits and the stadard KDL error detection to determine the feasibility of a pose. In it is NOT capable of collsion avoidance.

## Generalized Momentum Observer (GMO)
The GMO is a custom implemention of a GMO using KDL. It uses its own KDL representation decoupled from the controllers to detect collions and enforces protective stop using MoveIt!. 
The safety limits for the GMO are listed at the top of the source code and can be altered if needed. The GMO uses external wrench estimation (Forces and troques at the end efffector) to calculate the resulting acceleration experienced as a result of that at each joint. 
Using a simple first order observer it then detemines wether this acceleration crosses a predetermined threshold.
Addiotionally the GMO serves as an example of how to do force control WITHOUT a force/ torque sensor.  
