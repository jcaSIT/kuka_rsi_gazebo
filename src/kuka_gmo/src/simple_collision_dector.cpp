#include "simple_collision_dector.h"


simple_collision_dector::simple_collision_dector(std::string robot_description, moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr)
{
    move_group = move_group_ptr;
    ROS_INFO("Loading robotmodel from robot description: %s", robot_description.c_str());
    robot_model_loader = robot_model_loader::RobotModelLoader(robot_description);
    ROS_INFO("Loading kinematic model");
    kinematic_model = robot_model_loader.getModel();

    ROS_INFO("Generating kinematic state");
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    

    ROS_INFO("Generating KDL tree of robot from robot description: %s", robot_description.c_str());

    robot_model_KDL = KDL::Tree("base_link");
    if (!kdl_parser::treeFromParam("robot_description", robot_model_KDL))
    {
      ROS_ERROR("Failed to construct kdl tree");
      ROS_BREAK();
    }

    ROS_INFO("Generating forward dynamics");
    gravity.x(0.0);
    gravity.y(0.0);
    gravity.z(-9.82);
    robot_model_KDL.getChain("base_link","tool0",robot_chain);
    q = new KDL::JntArray(robot_chain.getNrOfJoints());
    qdot = new KDL::JntArray(robot_chain.getNrOfJoints());
    qdotdot = new KDL::JntArray(robot_chain.getNrOfJoints());
    jnt_torque = new KDL::JntArray(robot_chain.getNrOfJoints());
    jnt_torque_external = new KDL::JntArray(robot_chain.getNrOfJoints());
    jnt_torque_external_old = new KDL::JntArray(robot_chain.getNrOfJoints());
    collision_threshold = new KDL::JntArray(robot_chain.getNrOfJoints());
    wrench = new KDL::Wrench;
    B = new KDL::JntSpaceInertiaMatrix(robot_chain.getNrOfJoints());
    C = new KDL::JntArray(robot_chain.getNrOfJoints());
    G = new KDL::JntArray(robot_chain.getNrOfJoints());

    ROS_INFO("Generating KDL dynamic chain param");
    robot_dynamics = new KDL::ChainDynParam(robot_chain, gravity);
    ROS_INFO("Generating forward dynamics");

    extWrenchEstimator = new KDL::ChainExternalWrenchEstimator(robot_chain, gravity, 250, 1, 0.5);

    double threshold_arr[6] = {10, 10, 5, 5, 3, 3};
    for ( int i = 0; i < 6; i++)
    {
        collision_threshold->data(i) = threshold_arr[i];
    }

    updateExternalWrench();
}

void simple_collision_dector::add_new_msg(control_msgs::JointTrajectoryControllerState::ConstPtr new_msg)
{
    queue.push_front(new_msg);
    queue.resize(2);    
}

void simple_collision_dector::updateStateKDL()
{
    for(int i = 0; i < q->rows(); i ++)
    {
        control_msgs::JointTrajectoryControllerState::ConstPtr current_state = queue.front();
        q->data(i) = current_state->actual.positions.at(i);
        qdot->data(i) = current_state->actual.velocities.at(i);
        qdotdot->data(i) = current_state->desired.accelerations.at(i);
    }

}

void simple_collision_dector::updateExternalWrench()
{
    // update mass matrix B, coriolos matrix C and gravity matrix G
    robot_dynamics->JntToMass(*q, *B);
    robot_dynamics->JntToCoriolis(*q, *qdot, *C);
    robot_dynamics->JntToGravity(*q, *G);


    // use the DESIRED joint acceleration to calculate the corresponding joint torque
    jnt_torque->data = B->data * qdotdot->data + C->data * qdot->data + G ->data;

    // estimate the external wrench using the actual joint positions (q), velocities (q_dot) and the joint torque
    extWrenchEstimator->JntToExtWrench(*q, *qdot, *jnt_torque, *wrench);

    // get the estimated resulting jnt torques
    extWrenchEstimator->getEstimatedJntTorque(*jnt_torque_external);

}

void simple_collision_dector::feedbackStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    
    add_new_msg(msg);
    // Update joint postions veloceties and accelerations
    updateStateKDL();

    // Update the external wrench and the resulting external joint torque
    updateExternalWrench();
    

    // Ensure the queue is full.
    if(queue.back() != nullptr)
    {
        //The rate of change of the joint torque caused by an external wrench
        Eigen::VectorXd rate_of_change = (jnt_torque_external->data - jnt_torque_external_old->data) / (queue[0]->header.stamp.toSec() - queue[1]->header.stamp.toSec() );
        bool collision = false;
        std::cout << "\nEstimated rate of change: \n" << rate_of_change << '\n';
        
        for (int i = 0; i < jnt_torque->rows(); i++)
        {
            if(abs(rate_of_change(i)) > collision_threshold->data(i))
                collision = true;
        }

        jnt_torque_external_old->data = jnt_torque_external->data;


        if(collision)
        {
            move_group->stop();
            std::cout << "\nRate of change of external wrench.\n" << rate_of_change << '\n';
            ROS_ERROR("Experienced an external wrench that exceeded the safety limit (possible collision aborting)");
            ros::waitForShutdown();

        }
    }


}


simple_collision_dector::~simple_collision_dector()
{
}


