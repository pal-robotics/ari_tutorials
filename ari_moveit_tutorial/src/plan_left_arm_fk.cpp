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

// Std C++ headers
#include <string>
#include <vector>
#include <map>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_left_arm_fk");

  if ( argc < 5 )
  {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun ari_moveit_tutorial plan_left_arm_fk arm_left_1_joint arm_left_2_joint arm_left_3_joint arm_left_4_joint");
    ROS_INFO(" ");
    ROS_INFO("\twhere the list of arguments are the target values for the given joints");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }

  std::map<std::string, double> target_position;

  target_position["arm_left_1_joint"] = atof(argv[1]);
  target_position["arm_left_2_joint"] = atof(argv[2]);
  target_position["arm_left_3_joint"] = atof(argv[3]);
  target_position["arm_left_4_joint"] = atof(argv[4]);

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> arm_left_joint_names;
  //select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm_left("arm_left");
  //choose your preferred planner
  group_arm_left.setPlannerId("SBLkConfigDefault");

  arm_left_joint_names = group_arm_left.getJoints();

  group_arm_left.setStartStateToCurrentState();
  group_arm_left.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < arm_left_joint_names.size(); ++i)
    if ( target_position.count(arm_left_joint_names[i]) > 0 )
    {
      ROS_INFO_STREAM("\t" << arm_left_joint_names[i] << " goal position: " << target_position[arm_left_joint_names[i]]);
      group_arm_left.setJointValueTarget(arm_left_joint_names[i], target_position[arm_left_joint_names[i]]);
    }

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
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
