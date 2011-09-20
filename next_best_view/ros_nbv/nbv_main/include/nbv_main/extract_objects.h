/*
 * extract_objects.h
 *
 *  Created on: Dec 2, 2010
 *      Author: Christian Potthast
 */

#ifndef EXTRACT_OBJECTS_H_
#define EXTRACT_OBJECTS_H_

//#include <ros/ros.h>

#include <nbv_main/publish_topics.h>
#include <nbv_main/voxel_grid.h>
#include <nbv_main/planner.h>

#include <nbv_utilities/SendCloud.h>
#include <nbv_utilities/FindTable.h>
#include <tf/transform_listener.h>

#include <nbv_main/utilities.h>

class ExtractObjects{
  typedef pcl::PointXYZ Point;
  typedef pcl::PointXYZRGB PointRGB;

public:
  ExtractObjects(ros::NodeHandle& n);

  // Timer callback function
  void timerCallback(const ros::TimerEvent& event);
  // Service to get the input cloud
  bool sendCloud(nbv_utilities::SendCloud::Request &req, nbv_utilities::SendCloud::Response &res);
  // Service find table top
  bool findTable(nbv_utilities::FindTable::Request &req, nbv_utilities::FindTable::Response &res);
  // send kinect service
  bool kinectCloud(nbv_utilities::SendCloud::Request &req, nbv_utilities::SendCloud::Response &res);
  // extract the table top of the point cloud
  void getTableTop(pcl::PointCloud<Point>::ConstPtr &cloud, bool table_top);
  // segment out the objects from the point cloud
  void getObjects();
  // get the objects with limited field of view
  void getObjectsLimitedFOV(float direction, std::string name);
  // cluster the points to objects
  void labelObjects();
  // compute the sampling positions
  void samplingPositions(unsigned int n, double laser_tilt_mount_z, Eigen::Vector4f& centroid, float radius);
  std::vector<Eigen::Vector3f>
    samplingPositions2(unsigned int n, double laser_tilt_mount_z, Eigen::Vector4f& centroid, float radius);
  // create a voxel grid
  void initializeVoxelGrid();
  // set a new cloud in the voxel grid
  void setNewVoxelGridScan(pcl::PointCloud<PointRGB>::ConstPtr& cloud);
  // transform the clouds into the first cloud which is the referecen cloud
  void transformClouds(int& index);
  // visualize data
  void visualize();

private:
  // Node handle
  ros::NodeHandle node_handle_;
  //Timer for publish topics
  ros::Timer timer_;
  // Services
  ros::ServiceServer send_cloud_srv_;
  ros::ServiceServer find_table_srv_;
  ros::ServiceServer kinect_table_srv_;

  VoxelGrid* voxel_grid_;
  Planner planner_;
  Utilities utilities_;
  PublishTopics publish_topics_;

  std::vector<pcl::PointCloud<Point>::ConstPtr > input_clouds_;
  std::vector<pcl::PointCloud<Point>::ConstPtr > table_tops_;
  std::vector<pcl::PointCloud<Point>::ConstPtr > inv_table_tops_;
  std::vector<pcl::PointCloud<Point>::ConstPtr > objects_;
  std::vector<pcl::PointCloud<PointRGB>::ConstPtr > labeled_objects_;

  std::vector<float> colors_;

  std::vector<pcl::PointCloud<PointRGB> > seg_objects_;

  // centroid of the table top
  Eigen::Vector4f table_top_centroid_;
  // sampling positions
  std::vector<Eigen::Vector3f> sampling_positions_;
  std::vector<Eigen::Vector3f> full_sampling_positions_;
  std::vector<std::vector<float> > sampling_angle_;
  std::vector<std::vector<Eigen::Vector3f> > table_top_bbx_;

  // true if findTable was called
  bool init_;
  // reference cloud
  pcl::PointCloud<Point>::ConstPtr reference_cloud_;
  std::vector<Eigen::Vector3f> scan_poses_;

  // direction
  float direction_;

};

#endif /* EXTRACT_OBJECTS_H_ */
