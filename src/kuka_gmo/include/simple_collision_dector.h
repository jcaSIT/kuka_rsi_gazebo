
// Standard libs
#include <deque>

// ROS dependencies
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// Robot Model calculation
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include "chainexternalwrenchestimator.hpp"

#include <kdl_parser/kdl_parser.hpp>




class simple_collision_dector
{
private:

    // MoveIt "protective stop"
    moveit::planning_interface::MoveGroupInterfacePtr move_group;



    // Robot Model
    robot_model_loader::RobotModelLoader robot_model_loader;
    moveit::core::RobotModelPtr kinematic_model;
    moveit::core::RobotStatePtr kinematic_state;

    // KDL model
    KDL::Vector gravity;
    KDL::Tree robot_model_KDL;
    KDL::Chain robot_chain;
    KDL::ChainDynParam* robot_dynamics;

    // Dynamical Model for GMO on the 

    KDL::JntSpaceInertiaMatrix* B;
    KDL::JntArray* C;
    KDL::JntArray* G;
    KDL::JntArray* q;
    KDL::JntArray* qdot;
    KDL::JntArray* qdotdot;
    KDL::JntArray* qdotdot_external;
    KDL::JntArray* qdotdot_external_old;
    KDL::JntArray* collision_threshold;
    KDL::Wrench* wrench;

    // First order momentum observer
    KDL::ChainExternalWrenchEstimator* momentum_observer;

    // State Space Observer

    Eigen::MatrixXd* A_obs;
    Eigen::MatrixXd* B_obs;
    Eigen::MatrixXd* C_obs;
    Eigen::MatrixXd* F_obs;

    // Design parameters for SSO and KF
    Eigen::MatrixXd* A_ext;
    Eigen::MatrixXd* w_ext;


    bool this_is_bad_coding = true;





    
    // GMO message queue
    std::deque<control_msgs::JointTrajectoryControllerState::ConstPtr> queue;




public:
    simple_collision_dector(std::string robot_description, moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr);

    void add_new_msg(control_msgs::JointTrajectoryControllerState::ConstPtr new_msg);
    void feedbackStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
    void updateStateKDL();


    ~simple_collision_dector();
};