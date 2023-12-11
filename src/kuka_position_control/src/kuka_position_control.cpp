#include "kuka_position_control.h"


simple_position_controller::simple_position_controller(std::string robot_description)
{

    ROS_INFO("Generating KDL tree of robot from robot description: %s", robot_description.c_str());

    robot_model_KDL = KDL::Tree("base_link");
    if (!kdl_parser::treeFromParam(robot_description, robot_model_KDL))
    {
      ROS_ERROR("Failed to construct kdl tree");
      ROS_BREAK();
    }

    ROS_INFO("Generating forward dynamics");

    robot_model_KDL.getChain("base_link","tool0",robot_chain);
    q_current = new KDL::JntArray(robot_chain.getNrOfJoints());
    q_target = new KDL::JntArray(robot_chain.getNrOfJoints());

    fk_solver = new KDL::ChainFkSolverPos_recursive(robot_chain);
    ik_solver = new KDL::ChainIkSolverPos_LMA(robot_chain);


    joint_state_subscriber = node_handle.subscribe("joint_states", 1000, &simple_position_controller::controllerCallback, this);
    
    target_pose_subscriber = node_handle.subscribe("target_pose", 1000, &simple_position_controller::targetPoseCallback, this);
    
    joint_target_publisher = node_handle.advertise<std_msgs::Float64MultiArray>("pos_controller/command", 1,true);

}

simple_position_controller::~simple_position_controller(){
    if(std::uncaught_exception()) {
    ROS_ERROR("An error caused the postion controller to crash out!");
  }
}

void simple_position_controller::convertROSTFToKDLFrame(const geometry_msgs::TransformStamped& tf_msg, KDL::Frame& kdl_frame)
{
    // Convert the tf2::Transform to a KDL::Frame.
    kdl_frame = tf2::transformToKDL(tf_msg);
}


KDL::Frame convertStampedKDLFrameToKDLFrame(const tf2::Stamped<KDL::Frame>& stamped_frame)
{
    return KDL::Frame(stamped_frame.M, stamped_frame.p);
}


std_msgs::Float64MultiArray simple_position_controller::convertJntArrayToFloat64MultiArray(KDL::JntArray& jnt_array)
{
    std_msgs::Float64MultiArray float_array;
    for(unsigned int i=0; i < jnt_array.rows(); i++)
    {
        float_array.data.push_back(jnt_array(i));
    }
    return float_array;
}


void simple_position_controller::updateStateKDL(sensor_msgs::JointState joint_msg)
{
    for(int i = 0; i < q_current->rows(); i ++)
    {
        q_current->data(i) = joint_msg.position.at(i);
    }

}

int simple_position_controller::calculateForwardKinematics( KDL::Frame& result)
{

   // Calculate forward position kinematics
   return fk_solver->JntToCart(*q_current, result);
}

int simple_position_controller::calculateInverseKinematics(KDL::Frame& target_pose)
{
    target_pose_mutex.lock();
    int success = ik_solver->CartToJnt(*q_current, target_pose, *q_target);
    target_pose_mutex.unlock();
    return success;

}


void simple_position_controller::controllerCallback(const sensor_msgs::JointState joint_msg)
{

    int success = calculateInverseKinematics(target_pose);
    std_msgs::Float64MultiArray controller_msg = convertJntArrayToFloat64MultiArray(*q_target);
    joint_target_publisher.publish(controller_msg);

}


void simple_position_controller::targetPoseCallback(const geometry_msgs::TransformStamped& tf_msg)
{


    target_pose_mutex.lock();
    convertROSTFToKDLFrame(tf_msg, target_pose);
    target_pose_mutex.unlock();

}


