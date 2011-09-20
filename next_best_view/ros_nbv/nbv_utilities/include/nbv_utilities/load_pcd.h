/*
 * load_pcd.h
 *
 *  Created on: Dec 2, 2010
 *      Author: Christian Potthast
 */

#ifndef LOAD_PCD_H_
#define LOAD_PCD_H_

#include <sstream>
#include <stdio.h>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include "pcl/point_types.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nbv_utilities/SendCloud.h>
#include <nbv_utilities/FindTable.h>

class LoadPcd
{
public:
  LoadPcd(ros::NodeHandle& n);
  int load(std::string& file_name);

  sensor_msgs::PointCloud2 cloud_;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_;

  Eigen::Vector4f origin_;
  Eigen::Quaternionf orientation_;

private:
  ros::NodeHandle node_handle_;
  std::string tf_frame_;

};

#endif /* LOAD_PCD_H_ */
