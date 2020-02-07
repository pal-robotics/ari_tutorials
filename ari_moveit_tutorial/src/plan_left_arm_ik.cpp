/*
 * Copyright 2020 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
/** \author Jordi Pages. */

// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_left_arm_ik");

  if ( argc < 4 )
  {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun ari_moveit_tutorial plan_left_arm_ik  x y z  r p y");
    ROS_INFO(" ");
    ROS_INFO("\twhere the list of arguments specify the target pose of /arm_left_4_link expressed in /base_footprint");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }

  
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_footprint";
  goal_pose.pose.position.x = atof(argv[1]);
  goal_pose.pose.position.y = atof(argv[2]);
  goal_pose.pose.position.z = atof(argv[3]);
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(argv[4]), atof(argv[5]), atof(argv[6]));

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> arm_left_joint_names;
  //select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm_left("arm_left");
  //choose your preferred planner
  group_arm_left.setPlannerId("SBLkConfigDefault");
  group_arm_left.setPoseReferenceFrame("base_footprint");
  group_arm_left.setPoseTarget(goal_pose);

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_left.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_left.getPlanningFrame());

  
  group_arm_left.setStartStateToCurrentState();
  group_arm_left.setMaxVelocityScalingFactor(1.0);
  
  //Current position

  ROS_INFO_STREAM(group_arm_left.getCurrentPose());

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //set maximum time to find a plan
  group_arm_left.setPlanningTime(5.0);
  bool success = bool(group_arm_left.plan(my_plan));

  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();


  moveit::planning_interface::MoveItErrorCode e = group_arm_left.move();
  if (!bool(e))
    throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  spinner.stop();

  return EXIT_SUCCESS;
}
