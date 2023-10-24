
// Standard libs
#include <deque>

// ROS dependencies
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>




class simple_collision_dector
{
private:

    // Robot Model
    robot_model_loader::RobotModelLoader robot_model_loader;
    moveit::core::RobotModelPtr kinematic_model;
    moveit::core::RobotStatePtr kinematic_state;


    
    // GMO message queue
    std::deque<control_msgs::JointTrajectoryControllerState::ConstPtr> queue;




public:
    simple_collision_dector();
    simple_collision_dector(std::string robot_description);

    void add_new_msg(control_msgs::JointTrajectoryControllerState::ConstPtr new_msg);
    void feedbackStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);


    ~simple_collision_dector();
};