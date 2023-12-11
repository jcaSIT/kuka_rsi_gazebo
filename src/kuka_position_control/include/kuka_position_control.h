#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

#include <tf2_kdl/tf2_kdl.h>


#include <mutex>


class simple_position_controller
{
private:

    ros::NodeHandlePtr nh_ptr;

    KDL::Tree robot_model_KDL;
    KDL::Chain robot_chain;

    KDL::JntArray* q_current;
    KDL::JntArray* q_target;
    KDL::ChainIkSolverPos_LMA* ik_solver;
    KDL::ChainFkSolverPos_recursive* fk_solver;

    std::mutex target_pose_mutex;
    KDL::Frame target_pose;

    ros::NodeHandle node_handle;
    ros::Subscriber joint_state_subscriber;
    ros::Subscriber target_pose_subscriber;
    ros::Publisher joint_target_publisher;


public:
    /**
     * Constructor for the collision detector. It uses its own internal model of the robot to estimate collisions
     * \param robot_description The name of the robot destriction topic.
    */
    simple_position_controller(std::string robot_description);
    ~simple_position_controller();

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



};