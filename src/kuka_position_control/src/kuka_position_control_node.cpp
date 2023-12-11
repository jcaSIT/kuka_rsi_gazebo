#include <kuka_position_control.h>


int main(int argc, char** argv)
{

    ros::init(argc, argv, "kuka_gmo_node");

    
    simple_position_controller pos_controller("robot_description");



    static const std::string PLANNING_GROUP = "manipulator";

    // Generalized momentum oberserver based on https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8062635
    // Implemented using KDL and KDL parser


    ros::AsyncSpinner spinner(1);
    spinner.start();



    while (ros::ok())
    {
    /* code */
    }


    ros::shutdown();
    return 0;




}
