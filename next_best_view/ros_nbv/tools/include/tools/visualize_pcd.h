/*
 * visualize_pcd.h
 *
 *  Created on: May 18, 2011
 *      Author: potthast
 */

#ifndef VISUALIZE_PCD_H_
#define VISUALIZE_PCD_H_

#include <sstream>
#include <stdio.h>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <visualization_msgs/Marker.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class VisualizePCD
{
public:
  VisualizePCD(ros::NodeHandle& n);
  int load(std::string& file_name);
  void vis_camera_direction(std::string name, Eigen::Vector4f origin, Eigen::Quaternionf orientation);
  void publish();

private:

  ros::NodeHandle node_handle_;
  std::string tf_frame_;

  std::vector<ros::Publisher> publisher_;
  std::vector<sensor_msgs::PointCloud2> cloud_msg_;

  std::vector< PointCloud::Ptr > pc_xyz_;
  std::vector< PointCloudRGB::Ptr > pc_xyz_rgb_;
  std::vector<ros::Publisher> publisher_xyz_;
  std::vector<ros::Publisher> publisher_xyz_rgb_;
  std::vector<visualization_msgs::Marker> marker_msg_;
  std::vector<ros::Publisher> vis_;


};

#endif /* VISUALIZE_PCD_H_ */
