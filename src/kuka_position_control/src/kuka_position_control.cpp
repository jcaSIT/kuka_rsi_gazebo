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

    ROS_INFO_STREAM("Getting joint limits from robot description: " << robot_description);
    joint_limits_map = getJointLimitsFromURDF(robot_description);

    printJointLimits(joint_limits_map);


    ROS_INFO("Generating forward dynamics");

    robot_model_KDL.getChain("base_link","tool0",robot_chain);
    q_current = new KDL::JntArray(robot_chain.getNrOfJoints());
    q_target = new KDL::JntArray(robot_chain.getNrOfJoints());
    q_home = new KDL::JntArray(robot_chain.getNrOfJoints());
    q_home->data(1) = -1.57;
    q_home->data(2) = 1.57;
    q_home->data(4) = 1.57;



    // q_limit_min = new KDL::JntArray(robot_chain.getNrOfJoints());
    // q_limit_max = new KDL::JntArray(robot_chain.getNrOfJoints());



    fk_solver = new KDL::ChainFkSolverPos_recursive(robot_chain);
    ik_solver = new KDL::ChainIkSolverPos_LMA(robot_chain);


    joint_state_subscriber = node_handle.subscribe("joint_states", 1000, &simple_position_controller::controllerCallback, this);
    
    target_pose_subscriber = node_handle.subscribe("target_pose", 1000, &simple_position_controller::targetPoseCallback, this);
    
    joint_target_publisher = node_handle.advertise<std_msgs::Float64MultiArray>("pos_controller/command", 1,true);


    ROS_INFO("Postion controller running");

}

simple_position_controller::~simple_position_controller(){
    if(std::uncaught_exception()) {
    ROS_ERROR("An error caused the postion controller to crash out!");
  }
}

bool simple_position_controller::isDefault(const KDL::Frame& frame)
{
    return frame.M == KDL::Rotation::Identity() && frame.p == KDL::Vector::Zero();
}

void simple_position_controller::printJointLimits(const std::map<std::string, joint_limits_interface::JointLimits>& joint_limits_map)
{

    for(const auto& elem : joint_limits_map)
    {
        ROS_INFO_STREAM("Joint: " << elem.first 
                  << ", Lower Limit: " << elem.second.min_position 
                  << ", Upper Limit: " << elem.second.max_position 
                  << ", Max Effort: " << elem.second.max_effort 
                  << ", Max Velocity: " << elem.second.max_velocity);
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
    
    int success = 0;
    KDL::JntArray result;
    result.resize(robot_chain.getNrOfJoints());
    int i = 0;
    std::stringstream errorStream;
    std::string error_str;
    target_pose_mutex.lock();


    if(!isDefault(target_pose))
    {
        success = ik_solver->CartToJnt(*q_current, target_pose, result);
        
        if (success == 0)
        {
            for(const auto& elem : joint_limits_map)
            {
                if(result.data(i) < elem.second.min_position || result.data(i) > elem.second.max_position)
                {
                    success = 1; // Converged but degraded solution IE. the result is not a valid solution
                    errorStream << elem.first << " is out of bounds. Value: " << result.data(i) 
                                << ", Limits: (" << elem.second.min_position << ", " 
                                << elem.second.max_position << ").\n";
                }
                i++;
            }
        }
        error_str = errorStream.str();
        // if no error occured update the target joint position
        if(error_str.empty())
            q_target->data = result.data;
        else
            ROS_ERROR_STREAM_THROTTLE(1,"!!!JOINT LIMIT EXCEEDED!!\nThe target pose caused the following joints to exceed their limits:\n" << error_str);
    }
    else
    {
        q_target->data = q_home->data;
    }
    
    target_pose_mutex.unlock();
    return success;

}


void simple_position_controller::controllerCallback(const sensor_msgs::JointState joint_msg)
{

    int success = calculateInverseKinematics(target_pose);
    if( success == 0)
    {
        std_msgs::Float64MultiArray controller_msg = convertJntArrayToFloat64MultiArray(*q_target);
        joint_target_publisher.publish(controller_msg);
    }
    else
    {
        ROS_ERROR_STREAM_THROTTLE(1,"Invalid target pose. Inverse kinematics failed with code: " << success << " (" << ik_solver->strError(success) << ")\nResulting joint pose is infeasible. Last successful target joint pose:\n" << q_target->data);
    }


}


void simple_position_controller::targetPoseCallback(const geometry_msgs::TransformStamped& tf_msg)
{


    target_pose_mutex.lock();
    convertROSTFToKDLFrame(tf_msg, target_pose);
    target_pose_mutex.unlock();
    ROS_INFO_STREAM("Following new target:\n" << tf_msg);

}


std::map<std::string, joint_limits_interface::JointLimits> simple_position_controller::getJointLimitsFromURDF(const std::string& param_name)
{
    urdf::Model urdf_model;
    if (!urdf_model.initParam(param_name))
    {
        ROS_ERROR("Failed to parse URDF file from parameter: %s", param_name.c_str());
        return std::map<std::string, joint_limits_interface::JointLimits>();
    }

    std::map<std::string, joint_limits_interface::JointLimits> joint_limits_map;
    for(const auto& joint : urdf_model.joints_)
    {
        const auto& joint_name = joint.first;
        const auto& urdf_joint = joint.second;

        if(urdf_joint->type == urdf::Joint::UNKNOWN || urdf_joint->type == urdf::Joint::FIXED)
        {
            continue;
        }

        joint_limits_interface::JointLimits limits;
        if(joint_limits_interface::getJointLimits(urdf_joint, limits))
        {
            joint_limits_map[joint_name] = limits;
        }
    }

    return joint_limits_map;
}