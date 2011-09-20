/*
 * main_control.h
 *
 *  Created on: Dec 13, 2010
 *      Author: Christian Potthast
 */

#ifndef MAIN_CONTROL_H_
#define MAIN_CONTROL_H_

#include <sstream>
#include <stdio.h>
#include <fstream>

#include <ros/ros.h>
#include <nbv_utilities/BoolReturn.h>
#include <nbv_utilities/PcdFile.h>
#include <nbv_arm_planning/SendCloud.h>
#include <nbv_utilities/ArmPoseGoal.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_arm_msgs/MoveArmAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class MainControl
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
public:
  MainControl(ros::NodeHandle& n);

  void timerCallback(const ros::TimerEvent& event);

  void driveToInitPos();
  bool getCloud();
  void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
  bool getGoalPose();
  bool getGoalPoseArm();
  void saveCloud();
  void savePos();
  void sendGoalCmd();
  void sendArmGoalCmd();

  void setStatus(int s){status_=s;};


private:
  ros::NodeHandle node_handle_;
  ros::Timer timer_;
  ros::Subscriber robot_pose_sub_;
  sensor_msgs::PointCloud2 cloud_;

  geometry_msgs::PoseWithCovarianceStamped robot_pose_;
  geometry_msgs::PoseStamped goal_pose_;

  int num_;
  int status_;

  tf::StampedTransform transform_;

};

#endif /* MAIN_CONTROL_H_ */
