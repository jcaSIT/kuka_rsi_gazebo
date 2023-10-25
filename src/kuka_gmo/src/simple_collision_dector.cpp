#include "simple_collision_dector.h"


simple_collision_dector::simple_collision_dector(std::string robot_description)
{
    ROS_INFO("Loading robotmodel from robot description: %s", robot_description.c_str());
    robot_model_loader = robot_model_loader::RobotModelLoader(robot_description);
    ROS_INFO("Loading kinematic model");
    kinematic_model = robot_model_loader.getModel();

    ROS_INFO("Generating kinematic state");
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    

    ROS_INFO("Generating KDL tree of robot from robot description: %s", robot_description.c_str());
    if (!kdl_parser::treeFromString("/" + robot_description, robot_model_KDL))
    {
      ROS_ERROR("Failed to construct kdl tree");
      ROS_BREAK();
    }


    gravity.x(0.0);
    gravity.y(0.0);
    gravity.z(-9.82);
    robot_model_KDL.getChain("base_link","tool0",robot_chain);
    q = new KDL::JntArray(robot_chain.getNrOfJoints());
    qdot = new KDL::JntArray(robot_chain.getNrOfJoints());
    qdotdot = new KDL::JntArray(robot_chain.getNrOfJoints());
    B = new KDL::JntSpaceInertiaMatrix(robot_chain.getNrOfJoints());
    C = new KDL::JntArray(robot_chain.getNrOfJoints());
    G = new KDL::JntArray(robot_chain.getNrOfJoints());


    robot_dynamics = new KDL::ChainDynParam(robot_chain, gravity);

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
        q->data[i] = queue.at(i)->actual.positions[i];
        qdot->data[i] = queue.at(i)->actual.velocities[i];
        qdotdot->data[i] = queue.at(i)->actual.accelerations[i];
    }

}
void simple_collision_dector::feedbackStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    add_new_msg(msg);
    
    // Update joint postions veloceties and accelerations
    updateStateKDL();

    // update mass matrix B, coriolos matrix C and gravity matrix G
    robot_dynamics->JntToMass(*q, *B);
    robot_dynamics->JntToCoriolis(*q, *qdot, *C);
    robot_dynamics->JntToGravity(*q, *G);
    // std::cout << "MSG data: ";
    // for (auto i : queue.at(0)->error.positions)
    //     std::cout << i << " ";
    // std::cout << "\n";

    // std::cout << "q:" << q->rows() << " data: "  << q->data << "\n";

    // std::cout << "C matrix:\n" << C->data << "\n";

    // for (int i = 0; i < B->rows(); i++)
    //     for (int l = 0; l < B->columns(); l++)
    //     {
    //         std::cout << B->data() << " "; 
    //     }
        

    


}


simple_collision_dector::~simple_collision_dector()
{
}


