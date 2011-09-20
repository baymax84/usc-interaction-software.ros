/*
 * arm_pose_goal.h
 *
 *  Created on: Mar 16, 2011
 *      Author: potthast
 */

#ifndef ARM_POSE_GOAL_H_
#define ARM_POSE_GOAL_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <move_arm_msgs/MoveArmAction.h>
#include <move_arm_msgs/utils.h>


#include <nbv_utilities/ArmPoseGoal.h>

class ArmPoseGoal
{
public:
  ArmPoseGoal(ros::NodeHandle& n);

  bool armPoseGoalSrv(nbv_utilities::ArmPoseGoal::Request &req,
                      nbv_utilities::ArmPoseGoal::Response &res);


  move_arm_msgs::MoveArmGoal initialGoal();
  move_arm_msgs::MoveArmGoal startGoal();
  move_arm_msgs::MoveArmGoal testGoal();
  move_arm_msgs::MoveArmGoal waypoint_1();
  move_arm_msgs::MoveArmGoal waypoint_2();
  move_arm_msgs::MoveArmGoal waypoint_3();
  move_arm_msgs::MoveArmGoal waypoint_4();
  move_arm_msgs::MoveArmGoal waypoint_5();
  move_arm_msgs::MoveArmGoal waypoint_6();

  move_arm_msgs::MoveArmGoal motion_1();
  move_arm_msgs::MoveArmGoal motion_2();
  move_arm_msgs::MoveArmGoal motion_3();
  move_arm_msgs::MoveArmGoal motion_4();
  move_arm_msgs::MoveArmGoal motion_5();
  move_arm_msgs::MoveArmGoal motion_6();
  move_arm_msgs::MoveArmGoal motion_7();
  move_arm_msgs::MoveArmGoal motion_8();
  move_arm_msgs::MoveArmGoal motion_9();


  move_arm_msgs::MoveArmGoal init();



  bool sendGoal(move_arm_msgs::MoveArmGoal& goal);

private:
  ros::NodeHandle node_handle_;

  ros::ServiceServer arm_pose_goal_srv_;

};



#endif /* ARM_POSE_GOAL_H_ */
