/*
 * arm_pose_goal.cpp
 *
 *  Created on: Mar 16, 2011
 *      Author: potthast
 */

#include <nbv_utilities/arm_pose_goal.h>


ArmPoseGoal::ArmPoseGoal(ros::NodeHandle& n)
{
  node_handle_ = n;
  arm_pose_goal_srv_ = node_handle_.advertiseService("nbv_utilities/ArmPoseGoal", &ArmPoseGoal::armPoseGoalSrv, this);

  ROS_INFO("server started");

}

move_arm_msgs::MoveArmGoal ArmPoseGoal::initialGoal()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";
  pose.pose.position.x = 0.5;
  pose.pose.position.y = -0.5631;
  pose.pose.position.z = 0.7579;

  pose.pose.orientation.x = 0.022;
  pose.pose.orientation.y = -0.0035;
  pose.pose.orientation.z = 0.07789;
  pose.pose.orientation.w = -0.9966;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::startGoal()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";
  pose.pose.position.x = -0.0351;
  pose.pose.position.y = -0.9214;
  pose.pose.position.z = 1.1263;

  pose.pose.orientation.x = 0.0590;
  pose.pose.orientation.y = -0.2323;
  pose.pose.orientation.z = -0.3134;
  pose.pose.orientation.w = -0.91884;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::testGoal()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";
  pose.pose.position.x = 0.4028;
  pose.pose.position.y = 0.3902;
  pose.pose.position.z = 1.09423;

  pose.pose.orientation.x = 0.0092;
  pose.pose.orientation.y = -0.2725;
  pose.pose.orientation.z = 0.15268;
  pose.pose.orientation.w = -0.94990;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}


move_arm_msgs::MoveArmGoal ArmPoseGoal::init()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";
  pose.pose.position.x = -0.143;
  pose.pose.position.y = -0.875;
  pose.pose.position.z = 0.874;

  pose.pose.orientation.x = -0.039;
  pose.pose.orientation.y = 0.101;
  pose.pose.orientation.z = 0.177;
  pose.pose.orientation.w = 0.978;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::waypoint_1()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.400347;
  pose.pose.position.y = -0.694681;
  pose.pose.position.z = 1.30024;
  pose.pose.orientation.x = 0.384092;
  pose.pose.orientation.y = 0.484074;
  pose.pose.orientation.z = 0.249357;
  pose.pose.orientation.w = 0.745632;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::waypoint_2()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.390378;
  pose.pose.position.y = 0.051833;
  pose.pose.position.z = 1.30579;
  pose.pose.orientation.x = -0.403576;
  pose.pose.orientation.y = 0.437842;
  pose.pose.orientation.z = -0.229514;
  pose.pose.orientation.w = 0.769898;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::waypoint_3()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.193307;
  pose.pose.position.y = -0.549363;
  pose.pose.position.z = 1.29403;
  pose.pose.orientation.x = 0.45301;
  pose.pose.orientation.y = 0.420134;
  pose.pose.orientation.z = 0.255998;
  pose.pose.orientation.w = 0.743461;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::waypoint_4()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.688383;
  pose.pose.position.y = -0.061122;
  pose.pose.position.z = 1.2869;
  pose.pose.orientation.x = 0.170506;
  pose.pose.orientation.y = 0.682378;
  pose.pose.orientation.z = 0.168481;
  pose.pose.orientation.w = 0.690581;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::waypoint_5()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.369976;
  pose.pose.position.y = 0.331316;
  pose.pose.position.z = 1.31287;
  pose.pose.orientation.x = 0.0341;
  pose.pose.orientation.y = 0.404699;
  pose.pose.orientation.z = 0.015104;
  pose.pose.orientation.w = 0.913689;


  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::waypoint_6()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.224979;
  pose.pose.position.y = -0.090363;
  pose.pose.position.z = 1.36353;
  pose.pose.orientation.x = -0.302062;
  pose.pose.orientation.y = 0.517783;
  pose.pose.orientation.z = -0.201935;
  pose.pose.orientation.w = 0.77452;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}



move_arm_msgs::MoveArmGoal ArmPoseGoal::motion_1()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.285;
  pose.pose.position.y = -0.734;
  pose.pose.position.z = 1.176;
  pose.pose.orientation.x = -0.067;
  pose.pose.orientation.y = 0.258;
  pose.pose.orientation.z = 0.143;
  pose.pose.orientation.w = 0.953;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::motion_2()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.585;
  pose.pose.position.y = -0.517;
  pose.pose.position.z = 1.31;
  pose.pose.orientation.x = 0.384;
  pose.pose.orientation.y = 0.533;
  pose.pose.orientation.z = 0.296;
  pose.pose.orientation.w = 0.692;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::motion_3()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.5369;
  pose.pose.position.y = -0.3689;
  pose.pose.position.z = 1.250;
  pose.pose.orientation.x = 0.2895;
  pose.pose.orientation.y = 0.640;
  pose.pose.orientation.z = 0.2844;
  pose.pose.orientation.w = 0.6519;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::motion_4()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.5258;
  pose.pose.position.y = -0.2040;
  pose.pose.position.z = 1.265;
  pose.pose.orientation.x = 0.344;
  pose.pose.orientation.y = 0.535;
  pose.pose.orientation.z = 0.253;
  pose.pose.orientation.w = 0.728;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::motion_5()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.641;
  pose.pose.position.y = 0.057;
  pose.pose.position.z = 1.226;
  pose.pose.orientation.x = -0.022;
  pose.pose.orientation.y = 0.411;
  pose.pose.orientation.z = -0.295;
  pose.pose.orientation.w = 0.862;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::motion_6()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.669;
  pose.pose.position.y = -0.02364;
  pose.pose.position.z = 1.30045;
  pose.pose.orientation.x = -0.121175;
  pose.pose.orientation.y = 0.446093;
  pose.pose.orientation.z = -0.061104;
  pose.pose.orientation.w = 0.8846;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::motion_7()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.438358;
  pose.pose.position.y = 0.04037;
  pose.pose.position.z = 1.3959;
  pose.pose.orientation.x = 0.14879;
  pose.pose.orientation.y = 0.5540;
  pose.pose.orientation.z = 0.10138;
  pose.pose.orientation.w = 0.8128;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::motion_8()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.4125;
  pose.pose.position.y = 0.2433;
  pose.pose.position.z = 1.3047;
  pose.pose.orientation.x = 0.21708;
  pose.pose.orientation.y = 0.4494;
  pose.pose.orientation.z = 0.11356;
  pose.pose.orientation.w = 0.85907;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}

move_arm_msgs::MoveArmGoal ArmPoseGoal::motion_9()
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  motion_planning_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = "base_link";
  pose.link_name = "r_wrist_roll_link";

  pose.pose.position.x = 0.608;
  pose.pose.position.y = 0.121;
  pose.pose.position.z = 1.100;
  pose.pose.orientation.x = 0.121;
  pose.pose.orientation.y = 0.224;
  pose.pose.orientation.z = -0.620;
  pose.pose.orientation.w = 0.743;

  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;

  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;

  move_arm_msgs::addGoalConstraintToMoveArmGoal(pose, goal);

  return goal;
}



bool ArmPoseGoal::sendGoal(move_arm_msgs::MoveArmGoal& goal)
{
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_right_arm",true);
  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  if (node_handle_.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goal);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
      return false;
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }


  //ros::shutdown();

  return true;
}

bool ArmPoseGoal::armPoseGoalSrv(nbv_utilities::ArmPoseGoal::Request &req,
                                 nbv_utilities::ArmPoseGoal::Response &res)
{

  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_right_arm",true);
  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  move_arm_msgs::addGoalConstraintToMoveArmGoal(req.simple_pose_constraint, req.arm_pose_goal);

  if (node_handle_.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(req.arm_pose_goal);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
      {
        ROS_INFO("Action finished: %s",state.toString().c_str());
        res.result = res.SUCCEEDED;
      }
      else
      {
        ROS_INFO("Action failed: %s",state.toString().c_str());
        res.result = res.FAILED;
      }
    }
  }



  return true;
}


int main(int argc, char** argv)
{
  ros::init (argc, argv, "move_arm_pose_goal_test");
  ros::NodeHandle nh;

  ArmPoseGoal armPoseGoal(nh);

  move_arm_msgs::MoveArmGoal goal;

  if(argc == 2){
    std::string argument = argv[1];

    if( strcmp(argument.c_str(), "init") == 0 )
      goal = armPoseGoal.init();
    if( strcmp(argument.c_str(), "start") == 0 )
      goal = armPoseGoal.startGoal();
    if( strcmp(argument.c_str(), "test") == 0 )
      goal = armPoseGoal.testGoal();
    if( strcmp(argument.c_str(), "waypoint_1") == 0 ){
   //   goal = armPoseGoal.startGoal();
   //   armPoseGoal.sendGoal(goal);

      goal = armPoseGoal.waypoint_1();
      armPoseGoal.sendGoal(goal);

   //   goal = armPoseGoal.waypoint_3();
   //   armPoseGoal.sendGoal(goal);

   //   goal = armPoseGoal.testGoal();
   //   armPoseGoal.sendGoal(goal);


    }



    if( strcmp(argument.c_str(), "motion_1") == 0 ){
      goal = armPoseGoal.motion_1();
      armPoseGoal.sendGoal(goal);



      //goal = armPoseGoal.motion_3();
      //armPoseGoal.sendGoal(goal);


    }

  if( strcmp(argument.c_str(), "motion_2") == 0 ){
    goal = armPoseGoal.motion_2();
      armPoseGoal.sendGoal(goal);
  }
  if( strcmp(argument.c_str(), "motion_3") == 0 ){
    goal = armPoseGoal.motion_3();
      armPoseGoal.sendGoal(goal);
  }
  if( strcmp(argument.c_str(), "motion_4") == 0 ){
    goal = armPoseGoal.motion_4();
      armPoseGoal.sendGoal(goal);
  }

  if( strcmp(argument.c_str(), "motion_5") == 0 ){
    goal = armPoseGoal.motion_5();
      armPoseGoal.sendGoal(goal);
  }

  if( strcmp(argument.c_str(), "motion_6") == 0 ){
    goal = armPoseGoal.motion_6();
      armPoseGoal.sendGoal(goal);
  }

  if( strcmp(argument.c_str(), "motion_7") == 0 ){
    goal = armPoseGoal.motion_7();
      armPoseGoal.sendGoal(goal);
  }

  if( strcmp(argument.c_str(), "motion_8") == 0 ){
    goal = armPoseGoal.motion_8();
      armPoseGoal.sendGoal(goal);
  }
  if( strcmp(argument.c_str(), "motion_9") == 0 ){
    goal = armPoseGoal.motion_9();
      armPoseGoal.sendGoal(goal);
  }

    if( strcmp(argument.c_str(), "waypoint_2") == 0 )
      goal = armPoseGoal.waypoint_2();
    if( strcmp(argument.c_str(), "waypoint_3") == 0 )
      goal = armPoseGoal.waypoint_3();
    if( strcmp(argument.c_str(), "waypoint_4") == 0 )
      goal = armPoseGoal.waypoint_4();
    if( strcmp(argument.c_str(), "waypoint_5") == 0 )
      goal = armPoseGoal.waypoint_5();
    if( strcmp(argument.c_str(), "waypoint_6") == 0 )
      goal = armPoseGoal.waypoint_6();

    armPoseGoal.sendGoal(goal);

  }



  return 0;
}
