#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>

int main(int argc, char **argv){
  ros::init(argc, argv, "circle_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("/target_pose", 1000);

  ros::Rate rate(10); // 100 Hz
  double t = 0.0;
  double r = 0.3;
  geometry_msgs::TransformStamped center;
  center.header.stamp = ros::Time::now();
  center.header.frame_id = "world";
  center.child_frame_id = "target";
  center.transform.translation.x = 1.2; // Circular motion in X
  center.transform.translation.y = 0; // Circular motion in Y
  center.transform.translation.z = 0.5; // No motion in Z
  center.transform.rotation.x = 0;
  center.transform.rotation.y = 0.707;
  center.transform.rotation.z = 0;
  center.transform.rotation.w = 0.707; // No rotation

  pub.publish(center);

  for(int i = 0; i < 30; i++);
    rate.sleep();
  

  while(ros::ok() && t < 100){
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.child_frame_id = "target";
    msg.transform.translation.x = 1.2 + r * sin(t * 2 * M_PI / 10.0); // Circular motion in X
    msg.transform.translation.y = r * cos(t * 2 * M_PI / 10.0); // Circular motion in Y
    msg.transform.translation.z = 0.5; // No motion in Z
    msg.transform.rotation.x = 0;
    msg.transform.rotation.y = 0.707;
    msg.transform.rotation.z = 0;
    msg.transform.rotation.w = 0.707; // No rotation

    pub.publish(msg);
    
    t += 0.1; // Increase the time step
   
    ros::spinOnce();
    rate.sleep(); // Sleep to maintain the desired rate
  }

  if(ros::ok())
  {
    pub.publish(center);
  }

  return 0;
}
