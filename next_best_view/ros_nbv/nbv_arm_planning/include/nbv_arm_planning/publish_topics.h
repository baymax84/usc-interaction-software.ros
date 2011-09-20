/*
 * publish_topics.h
 *
 *  Created on: Dec 3, 2010
 *      Author: Christian Potthast
 */

#ifndef PUBLISH_TOPICS_H_
#define PUBLISH_TOPICS_H_

#include <nbv_arm_planning/utilities.h>

//#include <ros/ros.h>

// Messages
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
//#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
//#include <Eigen/StdVector>
#include "boost/multi_array.hpp"

class PublishTopics
{
  typedef boost::multi_array<int, 3> array_type;

public:
  PublishTopics () {};
  PublishTopics (ros::NodeHandle& n, std::string tf_frame);
  void
  newTopic (std::string &name, sensor_msgs::PointCloud2 pc);
  void
  newVisualizationMarker (visualization_msgs::Marker& marker_msg);
  void
  samplingPositionsMsg (std::vector<Eigen::Vector3f>& sampling_positions);
  void
  arrowMsg (std::string name, geometry_msgs::Pose pose);
  void
  robotPositionsMsg (Eigen::Vector3f robot_pose);
  void
  centroidPositionsMsg (Eigen::Vector4f& centroid);
  void
  voxelGridBbxMsg (std::vector<Eigen::Vector3f> table_top_bbx, Eigen::Vector3f bbx);
  void
  tableTopBbxMsg (std::vector<Eigen::Vector3f> & table_top_bbx);
  void
  sampledPosesMsg (geometry_msgs::PoseArray& poses);
  void
  PathMsg (std::string _topic_name, geometry_msgs::PoseArray& poses);
  void
  Publish ();

private:
  ros::NodeHandle node_handle_;
  std::string tf_frame_;
  std::vector<ros::Publisher> publisher_;
  std::vector<ros::Publisher> vis_publisher_;
  std::vector<sensor_msgs::PointCloud2> cloud_msg_;
  std::vector<visualization_msgs::Marker> marker_msg_;

  int robotPositionIndex_;

};

#endif /* PUBLISH_TOPICS_H_ */
