#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

std::vector<ros::Publisher> publishers;
std::vector<std_msgs::Float64> messages;

void posControllerCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

   for (int i = 0; i < 6; i++)
   {
       
       messages[i].data = msg->data[i];
       publishers[i].publish(messages[i]);
   }
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "pos_controller_subscriber");
   ros::NodeHandle n;

   ros::Subscriber sub = n.subscribe("pos_controller/command", 1000, posControllerCallback);

   
   for (int i = 1; i <= 6; i++)
   {
      std_msgs::Float64 joint_msg;
      std::string topic_name = "joint_a" + std::to_string(i) + "_position_controller/command";
      publishers.push_back(n.advertise<std_msgs::Float64>(topic_name, 1000));
      messages.push_back(joint_msg);
   }

   ros::spin();

   return 0;
}
