# Special Acknowledgements
I would like to extend my thanks to those who helped make this project possible. Firstly, I would like to thank my Professor, Takashi Yoshimi, for allowing me the opportunity to work in his lab and experience Japan. Secondly, I would like to extend a special thanks to Yoganata Kristanto for helping me debug, being a good sparring partner, and for being a good friend. 

Lastly, I would like to thank the [Scandinavia-Japan Sasakawa Foundation](https://sjsf.se/) for providing a scholarship to help fund my visit.

# Repo Overview
This repo contains an RSI interface and a simulation environment for the KUKA KR16 industrial robot. It was developed for YLab internal use.
The repo allows for simulation in Gazebo using either trajectory or position control. 

For installation, follow the guide in: [INSTALLATION.md](https://github.com/jcaSIT/kuka_rsi_gazebo/blob/master/INSTALLTION.md)

## Relevant Launch Files
Some helpful launch files to get you started!

### Gazebo Demo
To launch a simple demo of the Gazebo simulation, run:
```
roslaunch kuka_kr16_gazebo demo_gazebo.launch
```


### GMO Demo
To run the GMO demo, first launch the Gazebo demo then run:
```
roslaunch kuka_gmo kuka_gmo.launch
```
Then, using RVIZ, generate and execute a plan into the boxes placed by the robot. Upon collision, the robot should stop moving and the momentum observer will throw a collision error.

### Position Control Demo
To run the position control demo, run:
```
roslaunch kuka_position_control kuka_kr16_position_controller.launch
```
In a separate terminal, run:
```
rosrun kuka_position_control circle_motion
```
## Trajectory Controller
The trajectory controller is a MoveIt! Melodic implementation that is set up to work both as standard or as simulated in Gazebo. The trajectory controller has several planners available thanks to MoveIt! and is STRICTLY an offline controller.

## Position Controller
The position controller is a custom, bare-bones example of a position controller. It uses the URDF model to construct a KDL representation of the robot to calculate the inverse kinematics. It functions as a simple Servo Position server.
The position controller uses joint limits and the standard KDL error detection to determine the feasibility of a pose. However, it is NOT capable of collision avoidance.

## Generalized Momentum Observer (GMO)
The GMO is a custom implementation of a GMO using KDL. It uses its own KDL representation decoupled from the controllers to detect collisions and enforces a protective stop using MoveIt!. 
The safety limits for the GMO are listed at the top of the source code and can be altered if needed. The GMO uses external wrench estimation (forces and torques at the end effector) to calculate the resulting acceleration experienced as a result of that at each joint. 
Using a simple first-order observer, it then determines whether this acceleration crosses a predetermined threshold.
Additionally, the GMO serves as an example of how to do force control WITHOUT a force/torque sensor.
