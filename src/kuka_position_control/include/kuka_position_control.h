#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <urdf/model.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>


#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

#include <tf2_kdl/tf2_kdl.h>

#include <mutex>
#include <map>


class simple_position_controller
{
private:

    ros::NodeHandlePtr nh_ptr;

    KDL::Tree robot_model_KDL;
    KDL::Chain robot_chain;

    KDL::JntArray* q_current;
    KDL::JntArray* q_target;
    KDL::JntArray* q_limit_min;
    KDL::JntArray* q_limit_max;
    KDL::JntArray* q_home;

    std::map<std::string, joint_limits_interface::JointLimits> joint_limits_map;


    KDL::ChainIkSolverPos_LMA* ik_solver;
    KDL::ChainFkSolverPos_recursive* fk_solver;

    std::mutex target_pose_mutex;
    KDL::Frame target_pose;

    ros::NodeHandle node_handle;
    ros::Subscriber joint_state_subscriber;
    ros::Subscriber target_pose_subscriber;
    ros::Publisher joint_target_publisher;


    bool isDefault(const KDL::Frame& frame);


public:
    /**
     * Constructor for the collision detector. It uses its own internal model of the robot to estimate collisions
     * \param robot_description The name of the robot destriction topic.
    */
    simple_position_controller(std::string robot_description);
    ~simple_position_controller();

    void printJointLimits(const std::map<std::string, joint_limits_interface::JointLimits>& joint_limits_map);


    void convertROSTFToKDLFrame(const geometry_msgs::TransformStamped& tf_msg, KDL::Frame& kdl_frame);

    std_msgs::Float64MultiArray convertJntArrayToFloat64MultiArray(KDL::JntArray& jnt_array);

    void updateStateKDL(sensor_msgs::JointState joint_msg);

    /**
     * Constructor for the collision detector. It uses its own internal model of the robot to estimate collisions
     * \param robot_description The name of the robot destriction topic.
    */
    int calculateForwardKinematics(KDL::Frame& result);

    /**
     * Constructor for the collision detector. It uses its own internal model of the robot to estimate collisions
     * \param robot_description The name of the robot destriction topic.
    */
    int calculateInverseKinematics(KDL::Frame& target_pose);


    void controllerCallback(const sensor_msgs::JointState joint_msg);

    void targetPoseCallback(const geometry_msgs::TransformStamped& tf_msg);


    std::map<std::string, joint_limits_interface::JointLimits> getJointLimitsFromURDF(const std::string& param_name);



};