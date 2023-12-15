# Installation Guide
## Clone the repo and navigate into it (rename it if needed)
git clone https://github.com/jcaSIT/kuka_rsi_test.git
## Install Python Catkin Tools (allows the use of catkin build, clean, etc.)
sudo apt install python-catkin-tools 
## Install dependencies of some packages
rosdep install --from-paths src --ignore-src -r
## Install MoveIt (Melodic), MoveIt visual tools, ROS control, the Gazebo ROS controller interface, and the ROS joint controllers  
sudo apt install ros-melodic-moveit ros-melodic-ros-control ros-melodic-moveit-visual-tools ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-joint-trajectory-controller
## Build the workspace
catkin build
## Source the setup file (This is what allows ROS to find packages. i.e., this lets you run rosrun/roslaunch what we just built)
source devel/setup.sh
## Test the installation using the Gazebo demo
roslaunch kuka_kr16_gazebo demo_gazebo.launch
## Demo Instructions
An RVIZ and a Gazebo window should pop up. The Gazebo window is where the physical simulation runs. The RVIZ window visualizes the controller. Move the ball or drag the arrows in the RVIZ window to move the planned state. Press "Plan and Execute" to see the planner and controller in action. Both the robot in RVIZ and the Gazebo Robot should move. 

# IMPORTANT
The orange robot in RVIZ is JUST a representation of where the goal state of the user interface would plan to if you press "Plan and Execute". It is NOT the actual robot. If you control this using the MoveIt API movegroup like in the demo, it does not move (which is intentional).

