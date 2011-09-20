/*
 * utilities.h
 *
 *  Created on: Nov 10, 2010
 *      Author: Christian Potthast
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <iostream>

#include <ros/ros.h>
#include "pcl/filters/project_inliers.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>

#include <pcl/sample_consensus/sac_model_normal_plane.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


//#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <Eigen/StdVector>



class Utilities
{
  typedef pcl::PointXYZ Point;

public:
  Utilities();

  pcl::PointCloud<Point>::ConstPtr& extractTableTopNormals(pcl::PointCloud<Point>::ConstPtr &cloud, double distance_threshold=0.03, int max_iter=100, int k=50, double normal_distance_weight=0.1);
  pcl::PointCloud<Point>::ConstPtr& extractTableTop(pcl::PointCloud<Point>::ConstPtr &cloud, double distance_threshold=0.03, int max_iter=100);
  pcl::PointCloud<Utilities::Point>::ConstPtr& passthrough_filter(pcl::PointCloud<Point>::ConstPtr &cloud, std::string axis,double min, double max);
  pcl::PointCloud<Utilities::Point>::ConstPtr& statistical_outlier_removal(pcl::PointCloud<Point>::ConstPtr &cloud, int meanK=50, double StddevMulThresh=1.0);
  std::vector<pcl::PointCloud<Utilities::Point>::ConstPtr>& extract_euclidiean_cluster(pcl::PointCloud<Point>::ConstPtr &cloud, double tolerance=0.04, double min_cluster_size=50);
  pcl::PointCloud<Utilities::Point>::ConstPtr& mean_filter(std::vector<pcl::PointCloud<Point>::ConstPtr> &cloud);
  pcl::PointCloud<Utilities::Point>::ConstPtr& median_filter(std::vector<pcl::PointCloud<Point>::ConstPtr> &cloud);
  double avg_points_z(pcl::PointCloud<Point>::ConstPtr &cloud);
  int getClosestPositionAndErase(std::vector<Eigen::Vector3f>& sampling_positions,
                                 geometry_msgs::PoseWithCovarianceStamped robot_pose);
  // compute the bounding box of the point cloud (table top)
  std::vector<Eigen::Vector3f> fitTableTopBbx(pcl::PointCloud<Point>::ConstPtr &cloud);
  // transform the cloud using icp
  void transformCloud(pcl::PointCloud<Point>::ConstPtr& ref_cloud,
                      pcl::PointCloud<Point>::ConstPtr& trans_cloud,
                      pcl::PointCloud<Point>& cloud);
  Eigen::Vector4f computeCentroid(pcl::PointCloud<Point>::ConstPtr& cloud);

  Eigen::Vector2f computeNormal(Eigen::Vector2f p1, Eigen::Vector2f p2, float fov);

  pcl::PointCloud<Utilities::Point>::ConstPtr&
    get_inv_table_top_ptr(){return inv_table_top_ptr_;};
  pcl::ModelCoefficients::ConstPtr&
    get_plane_model_coefficients(){return table_coefficients_const_;};

  inline float
    getRGB (float r, float g, float b)
  {
    int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
    float rgb = *(float*)(&res);
    return (rgb);
  }

  pcl::PointCloud<Point> test;

private:
  pcl::PointCloud<Point>::ConstPtr table_top_ptr_;
  pcl::PointCloud<Point>::ConstPtr inv_table_top_ptr_;
  pcl::PointCloud<Point>::ConstPtr cloud_filtered_ptr_;
  pcl::PointCloud<Point>::ConstPtr sor_cloud_ptr_;
  std::vector<pcl::PointCloud<Utilities::Point>::ConstPtr> objects_ptr_vec_;
  pcl::PointCloud<Point>::ConstPtr mean_cloud_ptr_;
  pcl::PointCloud<Point>::ConstPtr median_cloud_ptr_;

  pcl::ModelCoefficients::ConstPtr table_coefficients_const_;

  geometry_msgs::PoseStamped goal_pose_;
};


#endif /* UTILITIES_H_ */
