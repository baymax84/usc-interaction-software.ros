/*
 * point_cloud_srv.hh
 *
 *  Created on: Jun 12, 2010
 *      Author: potthast
 */

#ifndef POINT_CLOUD_SRV_H_
#define POINT_CLOUD_SRV_H_

#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans.h"
#include "pr2_laser_snapshotter/BuildCloudAngle.h"
#include "pr2_msgs/SetPeriodicCmd.h"

// Messages
#include "sensor_msgs/PointCloud.h"
#include "pr2_msgs/LaserScannerSignal.h"

#include <boost/thread/mutex.hpp>

class PointCloudSrv
{
private:
  ros::Time laser_time_;
  boost::mutex laser_mutex_;
  ros::ServiceServer cloud_server_;
  ros::Subscriber sub_;
  ros::NodeHandle node_handle_;

public:

  PointCloudSrv(ros::NodeHandle& n);
  bool buildSingleSweepCloud(pr2_laser_snapshotter::BuildCloudAngle::Request &req,
                             pr2_laser_snapshotter::BuildCloudAngle::Response &res);
  void scannerSignalCallback(const pr2_msgs::LaserScannerSignalConstPtr& laser_scanner_signal);




};

#endif /* POINT_CLOUD_SRV_H_ */
