
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
    KDL::JntArray* jnt_torque;
    KDL::JntArray* jnt_torque_external;
    KDL::JntArray* jnt_torque_external_old;
    KDL::JntArray* collision_threshold;
    KDL::Wrench* wrench;

    // First order momentum observer
    KDL::ChainExternalWrenchEstimator* extWrenchEstimator;

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


    /**
     * Constructor for the collision detector. It uses its own internal model of the robot to estimate collisions
     * \param robot_description The name of the robot destriction topic.
     * \param move_group_ptr A pointer to a MoveIt move group. This is the system collisions will be detected for.

    */
    simple_collision_dector(std::string robot_description, moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr);
    ~simple_collision_dector();
 
    /**
     * The callback funtion needed to construct a subscriber to the appropriate feedback state topic. 
     * This function will handle all nessecary calculations for the collision detection.
     * Input:
     * \param msg A const pointer to a feedback state message.
    */
    void feedbackStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

private:
    /**
     * Simple queue insertion. The queue is just used to ensure adequate data. 
     * Can be expanded to handle delays by suspending dection until the data stream is solid again by emptying the queue
     * \param new_msg A const pointer to a feedback state message.
    */
    void add_new_msg(control_msgs::JointTrajectoryControllerState::ConstPtr new_msg);
    
    /**
     * The function updates the state representaion from the latest message in the queue
     * Updated parameters are:
     * q (actual value)
     * qdot (actual value)
     * qdotdot (DESIRED value)
    */
    void updateStateKDL();

      /**
     * The function updates the the external wrench by calculating the joint torque for the system from the desired acceleration.
     * It the estimates the external wrench using the previous calculations
    */
    void updateExternalWrench();

};