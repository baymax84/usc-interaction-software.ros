/*
 * driveToInitPos.h
 *
 *  Created on: Dec 6, 2010
 *      Author: Christian Potthast
 */

#ifndef DRIVE_TO_INI_TPOS_H_
#define DRIVE_TO_INIT_POS_H_

#include <iostream>
#include <string>
#include <cstdlib>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <sstream>
#include <stdio.h>
#include <fstream>


#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <nbv_utilities/FindTable.h>
#include <nbv_utilities/PcdFile.h>
#include <nbv_utilities/BoolReturn.h>

using namespace std;
using namespace boost::filesystem;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class DriveToInitPos
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

public:
  DriveToInitPos(ros::NodeHandle& n);
  // get the goal position from the exctract_obects class
  bool getGoalPose();
  // get the point cloud from the laser scanner
  bool getCloud();
  bool driveToInitPosSrv(nbv_utilities::BoolReturn::Request &req,
                         nbv_utilities::BoolReturn::Response &res);
  void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
  void sendGoalCmd();
  void saveCloud();
  void savePos();
  void getPointCloud(const sensor_msgs::PointCloud2 &msg);
  bool getKinectCloud();
  void transformCloud ();

  bool received_msg_;
  bool check_;

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber robot_pose_sub_;
  ros::ServiceServer drive_to_init_pos_srv_;

  sensor_msgs::PointCloud2 cloud_;
  ros::Subscriber sub_;

  geometry_msgs::PoseWithCovarianceStamped robot_pose_;
  geometry_msgs::PoseStamped goal_pose_;

  tf::StampedTransform transform_;
  tf::StampedTransform transform_kinect_;

  bool kinect_check;

};

#endif /* DRIVE_TO_INIT_POS_H_ */


