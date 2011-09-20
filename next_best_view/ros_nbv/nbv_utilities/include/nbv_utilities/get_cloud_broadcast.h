/*
 * get_cloud_broadcast.h
 *
 *  Created on: Apr 12, 2011
 *      Author: potthast
 */

#ifndef GET_CLOUD_BROADCAST_H_
#define GET_CLOUD_BROADCAST_H_

#include <ros/ros.h>
#include <nbv_utilities/PcdFile.h>

class GetCloudBroadcast
{
public:

  GetCloudBroadcast(ros::NodeHandle& n);
  // Timer callback function
  void timerCallback(const ros::TimerEvent& event);
  // get a cloud from the laser scanner service
  bool getCloudFromLaserScanner();

private:
  // ros node handle
  ros::NodeHandle node_handle_;
  //Timer for publish topics
  ros::Timer timer_;
  // laser scanner point cloud
  sensor_msgs::PointCloud2 cloud_;
  // cloud publisher
  ros::Publisher publisher_;

};


#endif /* GET_CLOUD_BROADCAST_H_ */
