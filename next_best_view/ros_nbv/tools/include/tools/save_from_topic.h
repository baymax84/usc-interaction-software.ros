/*
 * save_from_topic.h
 *
 *  Created on: May 18, 2011
 *      Author: potthast
 */

#ifndef SAVE_FROM_TOPIC_H_
#define SAVE_FROM_TOPIC_H_

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include "pcl/point_types.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <std_msgs/Header.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class SaveFromTopic
{
public:
  SaveFromTopic(ros::NodeHandle& n);

  void subscribe(std::string& topic_name, bool rgb, bool tf);
  tf::StampedTransform getCloudTransform(std_msgs::Header& pc_header);
  void getPointCloud(const PointCloud::ConstPtr& msg);
  void getPointCloudRGB(const PointCloudRGB::ConstPtr& msg);
  //void getPointCloudMsg(const sensor_msgs::PointCloud2 &msg);
  bool save();

  // set functions
  void setFileName(std::string f){file_name=f;};
  void setTfName(std::string tf){tf_name=tf;};


  Eigen::Vector4f origin_;
  Eigen::Quaternionf orientation_;
  bool saved;

private:
  ros::NodeHandle node_handle_;
  //std::string tf_frame_;
  //sensor_msgs::PointCloud2 point_cloud_;
  ros::Subscriber sub_;
  std::string file_name;
  std::string tf_name;
  tf::StampedTransform transform_;
  bool tf_;
};

#endif /* SAVE_FROM_TOPIC_H_ */
