/*
 * kinect_grab_one_pc_srv.h
 *
 *  Created on: Mar 17, 2011
 *      Author: potthast
 */

#ifndef KINECT_GRAB_ONE_PC_SRV_H_
#define KINECT_GRAB_ONE_PC_SRV_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nbv_utilities/PcdFile.h>

class KinectGrabOnePCSrv
{
public:
  KinectGrabOnePCSrv(ros::NodeHandle& n);

  bool kinect_grab_one_pc(nbv_utilities::PcdFile::Request &req, nbv_utilities::PcdFile::Response &res);
  void getPointCloud(const sensor_msgs::PointCloud2 &msg);
  void transformCloud();
  void saveCloud();
  bool getKinectCloud();

  //void kinectPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);

private:
  ros::NodeHandle node_handle_;
  ros::ServiceServer kinect_grab_one_pc_srv_;
  ros::Subscriber sub_;
  sensor_msgs::PointCloud2 point_cloud_;
  tf::StampedTransform transform_;

  ros::Subscriber kinect_pose_sub_;
  geometry_msgs::PoseWithCovarianceStamped kinect_pose_;
  tf::StampedTransform transform_kinect_;

  bool kinect_check;
  std::string frame_id_;

};

#endif /* KINECT_GRAB_ONE_PC_SRV_H_ */
