/*
 * point_cloud.hh
 *
 *  Created on: Jun 10, 2010
 *      Author: potthast
 */

#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_

#include <ros/ros.h>

// Service
#include <nbv_utilities/BoolReturn.h>
#include <nbv_utilities/PcdFile.h>
#include <pr2_laser_snapshotter/BuildCloudAngle.h>
#include <pr2_msgs/SetPeriodicCmd.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <tf/transform_listener.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/registration/transforms.h"

#include "sensor_msgs/point_cloud_conversion.h"


class PointCloud
{

private:
  ros::NodeHandle node_handle_;
  ros::ServiceServer point_cloud_srv_;
  ros::ServiceServer point_cloud_;

  ros::ServiceServer sendCloud_;


public:
  ros::Publisher scan_pub_;
  sensor_msgs::PointCloud cloud;
  sensor_msgs::PointCloud2 cloud2;

  PointCloud(ros::NodeHandle& n);
  bool broadcastCloud(nbv_utilities::PcdFile::Request &req, nbv_utilities::PcdFile::Response &res);
  bool broadcastPointCloud(nbv_utilities::PcdFile::Request &req, nbv_utilities::PcdFile::Response &res);
  bool sendCloud(nbv_utilities::PcdFile::Request &req,
                 nbv_utilities::PcdFile::Response &res);
  bool scan();
  bool stopTilt();

};


#endif /* POINT_CLOUD_H_ */
