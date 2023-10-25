/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */
#include <simple_collision_dector.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/JointTrajectoryControllerState.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "kuka_gmo_node");
  ros::NodeHandle node_handle;

  simple_collision_dector collision_detector("robot_description");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "manipulator";

  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ros::Subscriber feedback_subscriber = node_handle.subscribe("feedback_states", 1000, &simple_collision_dector::feedbackStateCallback, &collision_detector);
  
  // // Visualization
  // // ^^^^^^^^^^^^^
  // //
  // // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("link0");
  // visual_tools.deleteAllMarkers();
  // visual_tools.loadPlanningSceneMonitor();


  // // Remote control is an introspection tool that allows users to step through a high level script
  // // via buttons and keyboard shortcuts in RViz
  // visual_tools.loadRemoteControl();

  // // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // text_pose.translation().z() = 1.75;
  // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  // visual_tools.trigger();

  // // Getting Basic Information
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // We can print the name of the reference frame for this robot.
  // ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // // We can also print the name of the end-effector link for this group.
  // ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // // We can get a list of all the groups in the robot:
  // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
  //           std::ostream_iterator<std::string>(std::cout, ", "));

  // // Start the demo
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // // .. _move_group_interface-planning-to-pose-goal:
  // //
  // // Planning to a Pose goal
  // // ^^^^^^^^^^^^^^^^^^^^^^^
  // // We can plan a motion for this group to a desired pose for the
  // // end-effector.
  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 0.0;
  // target_pose1.orientation.x = 0.707;
  // target_pose1.orientation.y = 0.0;
  // target_pose1.orientation.z = 0.707;

  // target_pose1.position.x = 1.378;
  // target_pose1.position.y = 0.000;
  // target_pose1.position.z = 1.005;
  // move_group.setPoseTarget(target_pose1);

  // // Now, we call the planner to compute the plan and visualize it.
  // // Note that we are just planning, not asking move_group
  // // to actually move the robot.
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // move_group.setPlannerId("RRTConnectkConfigDefault");


  // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // move_group.execute(my_plan.trajectory_);

  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // // Visualizing plans
  // // ^^^^^^^^^^^^^^^^^
  // // We can also visualize the plan as a line with markers in RViz.
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // // Moving to a pose goal
  // // ^^^^^^^^^^^^^^^^^^^^^
  // //
  // // Moving to a pose goal is similar to the step above
  // // except we now use the move() function. Note that
  // // the pose goal we had set earlier is still active
  // // and so the robot will try to move to that goal. We will
  // // not use that function in this tutorial since it is
  // // a blocking function and requires a controller to be active
  // // and report success on execution of a trajectory.

  // /* Uncomment below line when working with a real robot */
  // // Move the real robot
  // move_group.move();

  while (ros::ok())
  {
    /* code */
  }
  

  ros::shutdown();
  return 0;
}
