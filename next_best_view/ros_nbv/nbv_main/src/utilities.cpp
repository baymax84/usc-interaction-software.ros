/*
 * utilities.cpp
 *
 *  Created on: Nov 10, 2010
 *      Author: Christian Potthast
 */

#include <nbv_main/utilities.h>

Utilities::Utilities(){


}


pcl::PointCloud<Utilities::Point>::ConstPtr&
  Utilities::extractTableTop(pcl::PointCloud<Point>::ConstPtr &cloud, double distance_threshold, int max_iter)
{

  pcl::PointCloud<Point> table_top;

  //planaer model segmentation
  //double distance_threshold_ = 0.03;  //0.03
  //int max_iter = 250; //1000

  // Obtain the plane inliers and coefficients
  pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr table_coefficients (new pcl::ModelCoefficients ());

  pcl::SACSegmentation<Point> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (max_iter);  //1000
  seg.setDistanceThreshold (distance_threshold);
  seg.setProbability (0.99);
  seg.setInputCloud(cloud);
  seg.segment(*table_inliers, *table_coefficients);


  //pcl::PointIndices::ConstPtr table_inliers_const;
  //table_inliers_const.reset (new pcl::PointIndices (table_inliers));
  //table_coefficients_const_.reset (new pcl::ModelCoefficients (table_coefficients));
  table_coefficients_const_ = table_coefficients;

  // extract planar inlier
  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.setNegative (false);
  extract.filter (table_top);
  table_top_ptr_ = boost::make_shared<pcl::PointCloud<Point> >(table_top);

  pcl::PointCloud<Point> inv_table_top;
  extract.setNegative (true);
  extract.filter (inv_table_top);
  inv_table_top_ptr_ = boost::make_shared<pcl::PointCloud<Point> >(inv_table_top);

  return table_top_ptr_;

}


pcl::PointCloud<Utilities::Point>::ConstPtr&
  Utilities::extractTableTopNormals(pcl::PointCloud<Point>::ConstPtr &cloud,
                                    double distance_threshold, int max_iter,
                                    int k, double normal_distance_weight)
{
  pcl::PointCloud<Point> table_top;

  // estimate point normals
  pcl::PointCloud<pcl::Normal> cloud_normals;
  //int k = 50;  //50

  pcl::NormalEstimation<Point, pcl::Normal> ne;
  pcl::KdTreeFLANN<Point>::Ptr normals_tree = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
  ne.setKSearch (k);
  ne.setSearchMethod(normals_tree);
  ne.setInputCloud(cloud);
  ne.compute (cloud_normals);


//  for(int i=0; i<cloud_normals.size(); i++){
//    std::cout << "x: " << cloud_normals[i]->
//  }

  pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_const;
  cloud_normals_const.reset (new pcl::PointCloud<pcl::Normal> (cloud_normals));

  //planaer model segmentation
  //double normal_distance_weight_ = 0.1;
  //double distance_threshold_ = 0.015;  //0.03
  //int max_iter = 100; //1000

  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg;

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setEpsAngle(0.1);
  seg.setAxis(Eigen::Vector3f(0.0,0.0,1.0));
  seg.setNormalDistanceWeight (normal_distance_weight);
  seg.setMaxIterations (max_iter);  //1000
  seg.setDistanceThreshold (distance_threshold);
  seg.setProbability (0.99);
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals_const);

  // Obtain the plane inliers and coefficients
  //pcl::PointIndices table_inliers;
  pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices ());
  //pcl::PointIndices::Ptr table_inliers;
  //pcl::ModelCoefficients table_coefficients;
  //pcl::ModelCoefficients::Ptr table_coefficients;
  pcl::ModelCoefficients::Ptr table_coefficients (new pcl::ModelCoefficients ());


  seg.segment(*table_inliers, *table_coefficients);

  //int cloud_size = cloud->width * cloud->height;
  //ROS_INFO("ExtractObjects - sendCloud: size of the cloud: %d", cloud_size );


  pcl::PointIndices::ConstPtr table_inliers_const;
  //table_inliers_const.reset (new pcl::PointIndices (table_inliers));
  //table_coefficients_const_.reset (new pcl::ModelCoefficients (table_coefficients));
  table_coefficients_const_ = table_coefficients;

  // extract planar inlier
  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.setNegative (false);
  extract.filter (table_top);
  table_top_ptr_ = boost::make_shared<pcl::PointCloud<Point> >(table_top);

  pcl::PointCloud<Point> inv_table_top;
  extract.setNegative (true);
  extract.filter (inv_table_top);
  inv_table_top_ptr_ = boost::make_shared<pcl::PointCloud<Point> >(inv_table_top);

  return table_top_ptr_;

}

pcl::PointCloud<Utilities::Point>::ConstPtr&
Utilities::passthrough_filter(pcl::PointCloud<Point>::ConstPtr &cloud, std::string axis,double min, double max){

    pcl::PointCloud<Point> cloud_filtered;
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName (axis);
    pass.setFilterLimits (min, max);
    //pass.setFilterLimitsNegative (true);
    pass.filter (cloud_filtered);
    cloud_filtered_ptr_ = boost::make_shared<pcl::PointCloud<Point> >(cloud_filtered);

    return cloud_filtered_ptr_;

}

pcl::PointCloud<Utilities::Point>::ConstPtr&
Utilities::statistical_outlier_removal(pcl::PointCloud<Point>::ConstPtr &cloud, int meanK, double StddevMulThresh){

  pcl::PointCloud<Point> sor_cloud;
  pcl::StatisticalOutlierRemoval<Point> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK (meanK);
  sor.setStddevMulThresh (StddevMulThresh);
  sor.filter (sor_cloud);

  sor_cloud_ptr_ = boost::make_shared<pcl::PointCloud<Point> >(sor_cloud);

  return sor_cloud_ptr_;

}

std::vector<pcl::PointCloud<Utilities::Point>::ConstPtr>&
Utilities::extract_euclidiean_cluster(pcl::PointCloud<Point>::ConstPtr &cloud, double tolerance, double min_cluster_size)
{

  objects_ptr_vec_.clear();
  std::vector<pcl::PointIndices> clusters;

  pcl::KdTreeFLANN<Point>::Ptr kd_tree = boost::make_shared<pcl::KdTreeFLANN<Point> > ();;
  pcl::EuclideanClusterExtraction<Point> ece;

  ece.setSearchMethod(kd_tree);
  ece.setClusterTolerance(tolerance);
  ece.setMinClusterSize(min_cluster_size);
  ece.setInputCloud(cloud);
  ece.extract(clusters);

  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud(cloud);
  extract.setNegative (false);

  for(unsigned int ii=0; ii<clusters.size(); ii++)
  {
    //pcl::PointIndices::Ptr inliers;

   // inliers = boost::make_shared<pcl::PointIndices> (clusters[ii]);
    extract.setIndices(boost::make_shared<pcl::PointIndices> (clusters[ii]));
    //extract.setIndices(inliers);
    pcl::PointCloud<Point> object;
    extract.filter(object);
    pcl::PointCloud<Point>::ConstPtr object_ptr;
    object_ptr = boost::make_shared<pcl::PointCloud<Point> >(object);
    objects_ptr_vec_.push_back(object_ptr);
  }

  return objects_ptr_vec_;


}

pcl::PointCloud<Utilities::Point>::ConstPtr&
  Utilities::mean_filter(std::vector< pcl::PointCloud<Point>::ConstPtr > &cloud){

  // We assume all the point clouds have the same number of points, and that they have the same order
  // If that isn't the case, this will produce random results
  unsigned int n_points = cloud[0]->size();
  unsigned int n_measurements = cloud.size();

  pcl::PointCloud<Point> cloud_filtered;
  cloud_filtered.header = cloud[0]->header;
  // zzz Fix later: cloud_filtered.points.size = cloud[0].points.size();
  cloud_filtered.is_dense = cloud[0]->is_dense;

  // zzz should be fixed! do not copie the entire point cloud !!!
  cloud_filtered = *cloud[0];

  std::vector<Point> points(n_measurements);
  for(unsigned int pp=0; pp<n_points; pp++){
    // Filter pixel by pixel
    for(unsigned int ii=0; ii<n_measurements; ii++){
      points[ii] = cloud[ii]->points[pp];
    }
    // Now filter

    // Simple mean filter
    Point point_mean = Point(0,0,0);
    for(unsigned int ii=0; ii<n_measurements; ii++){
      point_mean.x += points[ii].x/n_measurements;
      point_mean.y += points[ii].y/n_measurements;
      point_mean.z += points[ii].z/n_measurements;
    }

    // Done filtering for this pixel
    cloud_filtered.points[pp] = point_mean;

  }

  mean_cloud_ptr_ = boost::make_shared<pcl::PointCloud<Point> >(cloud_filtered);
  return mean_cloud_ptr_;

}

pcl::PointCloud<Utilities::Point>::ConstPtr&
  Utilities::median_filter(std::vector< pcl::PointCloud<Point>::ConstPtr > &cloud){

  // We assume all the point clouds have the same number of points, and that they have the same order
  // If that isn't the case, this will produce random results
  unsigned int n_points = cloud[0]->size();
  unsigned int n_measurements = cloud.size();
  unsigned int median_index = floor(n_measurements/2);

  std::vector<double> points_x(n_measurements);
  std::vector<double> points_y(n_measurements);
  std::vector<double> points_z(n_measurements);

  pcl::PointCloud<Point> cloud_filtered;
  cloud_filtered.header = cloud[0]->header;
  // zzz Fix later: cloud_filtered.points.size = cloud[0].points.size();
  cloud_filtered.is_dense = cloud[0]->is_dense;


  cloud_filtered = *cloud[0];

  std::vector<Point> points(n_measurements);
  for(unsigned int pp=0; pp<n_points; pp++){
    // Filter pixel by pixel
    for(unsigned int ii=0; ii<n_measurements; ii++){
      points[ii] = cloud[ii]->points[pp];
    }
    // Now filter

    // median filter
    Point point_median = Point(0,0,0);
    for(unsigned int ii=0; ii<n_measurements; ii++){
      points_x[ii] = points[ii].x;
      points_y[ii] = points[ii].y;
      points_z[ii] = points[ii].z;
    }

    sort(points_x.begin(), points_x.end());
    sort(points_y.begin(), points_y.end());
    sort(points_z.begin(), points_z.end());

    point_median.x = points_x[median_index];
    point_median.y = points_y[median_index];
    point_median.z = points_z[median_index];


    // Done filtering for this pixel
    cloud_filtered.points[pp] = point_median;

  }

  median_cloud_ptr_ = boost::make_shared<pcl::PointCloud<Point> >(cloud_filtered);
  return median_cloud_ptr_;

}

double Utilities::avg_points_z(pcl::PointCloud<Point>::ConstPtr &cloud){
  // average over the 'z' coordinate of the table top plane
  // get the 'x' nearest front
  unsigned int n_points = (unsigned int)cloud->points.size();
  double z_coord = 0.0;
  double x_coord = cloud->points[0].x;
  for (size_t i = 0; i < n_points ; ++i)
  {
    z_coord += cloud->points[i].z;

    if (x_coord > cloud->points[i].x)
      x_coord = cloud->points[i].x;
  }
  z_coord = z_coord/n_points;

  return z_coord;
}

int Utilities::getClosestPositionAndErase(std::vector<Eigen::Vector3f>& sampling_positions,
                                          geometry_msgs::PoseWithCovarianceStamped robot_pose )
{
  float dist = 100.0;
  int index = 0;

  for(unsigned int ii=0; ii<sampling_positions.size(); ii++)
  {
    float tmp_dist = sqrt(pow(robot_pose.pose.pose.position.x - sampling_positions[ii].x(),2) +
                          pow(robot_pose.pose.pose.position.y - sampling_positions[ii].y(),2) +
                          pow(robot_pose.pose.pose.position.z - sampling_positions[ii].z(),2));
    if(tmp_dist<dist){
      dist = tmp_dist;
      index=ii;
    }
  }

  // erase the sample point from the list, which is closest to
  // the scan position of possible sample positions.
  sampling_positions.erase(sampling_positions.begin() + index);

  return index;
}

std::vector<Eigen::Vector3f> Utilities::fitTableTopBbx(pcl::PointCloud<Point>::ConstPtr &cloud)
{

  std::vector<Eigen::Vector3f> table_top_bbx;

  // Project points onto the table plane
  pcl::ProjectInliers<Point> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  pcl::PointCloud<Point> projected_cloud;
  proj.setInputCloud(cloud);
  proj.setModelCoefficients(table_coefficients_const_);
  proj.filter(projected_cloud);

  // store the table top plane parameters
  Eigen::Vector3f plane_normal;
  plane_normal.x() = table_coefficients_const_->values[0];
  plane_normal.y() = table_coefficients_const_->values[1];
  plane_normal.z() = table_coefficients_const_->values[2];
  // compute an orthogonal normal to the plane normal
  Eigen::Vector3f v = plane_normal.unitOrthogonal();
  // take the cross product of the two normals to get
  // a thirds normal, on the plane
  Eigen::Vector3f u = plane_normal.cross(v);

  // project the 3D point onto a 2D plane
  std::vector<cv::Point2f> points;
  // choose a point on the plane
  Eigen::Vector3f p0(projected_cloud.points[0].x,
                      projected_cloud.points[0].y,
                      projected_cloud.points[0].z);
  for(unsigned int ii=0; ii<projected_cloud.points.size(); ii++)
  {
    Eigen::Vector3f p3d(projected_cloud.points[ii].x,
                         projected_cloud.points[ii].y,
                         projected_cloud.points[ii].z);

    // subtract all 3D points with a point in the plane
    // this will move the origin of the 3D coordinate system
    // onto the plane
    p3d = p3d - p0;

    cv::Point2f p2d;
    p2d.x = p3d.dot(u);
    p2d.y = p3d.dot(v);
    points.push_back(p2d);
  }

  cv::Mat points_mat(points);
  cv::RotatedRect rrect = cv::minAreaRect(points_mat);
  cv::Point2f rrPts[4];
  rrect.points(rrPts);

  //store the table top bounding points in a vector
  for(unsigned int ii=0; ii<4; ii++)
  {
    Eigen::Vector3f pbbx(rrPts[ii].x*u + rrPts[ii].y*v + p0);
    table_top_bbx.push_back(pbbx);
  }
  Eigen::Vector3f center(rrect.center.x*u + rrect.center.y*v + p0);
  table_top_bbx.push_back(center);

  return table_top_bbx;

}

void Utilities::transformCloud(pcl::PointCloud<Point>::ConstPtr& ref_cloud,
                               pcl::PointCloud<Point>::ConstPtr& trans_cloud,
                               pcl::PointCloud<Point>& cloud)
{

  pcl::IterativeClosestPoint<Point , Point> icp;
  icp.setInputCloud(trans_cloud);
  icp.setInputTarget(ref_cloud);

  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-8);

  pcl::PointCloud<Point> tmp_cloud;
  icp.align(tmp_cloud);

  Eigen::Matrix4f T;
  T = icp.getFinalTransformation();

  pcl::transformPointCloud(cloud, tmp_cloud, T);
  pcl::transformPointCloud(*trans_cloud, test, T);

  cloud.points = tmp_cloud.points;
}

Eigen::Vector4f Utilities::computeCentroid(pcl::PointCloud<Point>::ConstPtr& cloud)
{
  Eigen::Vector4f min, max;
  pcl::getMinMax3D(*cloud, min, max);
  Eigen::Vector4f centroid;
  centroid.x() = (min.x() + max.x()) /2;
  centroid.y() = (min.y() + max.y()) /2;
  centroid.z() = (min.z() + max.z()) /2;

  return centroid;
}

Eigen::Vector2f Utilities::computeNormal(Eigen::Vector2f p1, Eigen::Vector2f p2, float fov)
{
  float theta = fov * M_PI/180;
  Eigen::Matrix2f R;
  R << cos(theta), -sin(theta),
       sin(theta),  cos(theta);

  Eigen::Vector2f p2f(0.0,0.0);
  p2f = R*(p1-p2) + p2;
  Eigen::Vector2f p(p2f.x(), p2f.y());

  Eigen::Vector2f pp = p-p2;
  Eigen::Vector2f normal = pp.unitOrthogonal();

  return normal;
}

