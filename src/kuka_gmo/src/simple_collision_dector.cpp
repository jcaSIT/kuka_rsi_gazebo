#include "simple_collision_dector.h"


simple_collision_dector::simple_collision_dector()
{
    robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
    kinematic_model = robot_model_loader.getModel();
}

simple_collision_dector::simple_collision_dector(std::string robot_description)
{
    robot_model_loader = robot_model_loader::RobotModelLoader(robot_description);
    kinematic_model = robot_model_loader.getModel();

    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));

    

}

void simple_collision_dector::add_new_msg(control_msgs::JointTrajectoryControllerState::ConstPtr new_msg)
{
    queue.push_front(new_msg);
    queue.resize(2);    
}

void simple_collision_dector::feedbackStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    
    add_new_msg(msg);
    for (auto i : queue.at(0)->error.positions)
        std::cout << i << " ";
    std::cout << "\n";

}


simple_collision_dector::~simple_collision_dector()
{
}  