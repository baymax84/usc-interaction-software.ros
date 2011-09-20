/*
 * point_cloud.cpp
 *
 *  Created on: Jun 10, 2010
 *      Author: Christian Potthast
 */

#include "nbv_utilities/point_cloud.h"

PointCloud::PointCloud(ros::NodeHandle& n)
{
  node_handle_ = n;
  point_cloud_srv_ = node_handle_.advertiseService("nbv_utilities/PointCloud", &PointCloud::broadcastCloud, this);
//  point_cloud_ = n_.advertiseService("point_cloud/cloud", &PointCloud::broadcastPointCloud, this);

  sendCloud_ = node_handle_.advertiseService("nbv_utilities/SendCloud", &PointCloud::sendCloud, this);

}


bool PointCloud::sendCloud(nbv_utilities::PcdFile::Request &req,
                           nbv_utilities::PcdFile::Response &res)
{
  if(req.req == true)
  {
    scan();
  }

  res.cloud = cloud2;


  return true;
}



bool PointCloud::broadcastCloud(nbv_utilities::PcdFile::Request &req,
                                nbv_utilities::PcdFile::Response &res)
{

  if(req.req == true)
  {
    scan();
    stopTilt();

  }

  if(req.req == false)
  {
    ROS_INFO("stop laser tilt");
    stopTilt();
  }

  res.cloud = cloud2;


  return true;

}

bool PointCloud::broadcastPointCloud(nbv_utilities::PcdFile::Request &req,
                                     nbv_utilities::PcdFile::Response &res)
{
  res.cloud = cloud2;

  return true;
}

bool PointCloud::scan()
{
  pr2_laser_snapshotter::BuildCloudAngle::Request snap_req;
  pr2_laser_snapshotter::BuildCloudAngle::Response snap_res;

  snap_req.angle_begin = -0.8;
  snap_req.angle_end = 0.5;
  snap_req.duration = 20.0;

  if (!ros::service::call("point_cloud_srv/single_sweep_cloud", snap_req, snap_res)){
    ROS_ERROR("PointCloud: error setting laser snapshotter service");
    return false;

  }else{
    ROS_INFO("PointCloud : laser snapshotter with angle_begin %f, angle_end %f and duration %f",
             snap_req.angle_begin, snap_req.angle_end, snap_req.duration);
  }

  ROS_INFO("PointCloud: received point cloud of size %i from point cloud assembler", (int)snap_res.cloud.points.size());

  stopTilt();





/*
  Eigen::Quaternionf originr;
  Eigen::Vector3f origino;

  origino.x() =0.0;
  origino.y() = 0.0;
  origino.z() =0.0;

  originr.x() = 0.0;
  originr.y() = 0.0;
  originr.z() = 0.0;
  originr.w() = 1.0;

  Eigen::Quaternionf newr;
  Eigen::Vector3f newo;

  newo = origino - offset_;



  tf::TransformListener listener;
  tf::StampedTransform transform;
*/

  cloud = snap_res.cloud;

/*
  double theta = 2.35619;
  Eigen::Vector3f C0(0,0,0);
  Eigen::Vector3f C1(2.51594, -1.04586, 0.0);
  Eigen::Vector3f C = C1-C0;

  Eigen::Matrix4f T;
  T << cos(theta), -sin(theta), 0, C.x(),
       sin(theta),  cos(theta), 0, C.y(),
       0, 0, 1, C.z(),
       0, 0, 0, 1;
  std::cout << T;

  for(int i=0; i < (int)cloud.points.size(); i++){
    Eigen::Vector4f p(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1);
    Eigen::Vector4f t = T*p ;

    cloud.points[i].x = t.x();
    cloud.points[i].y = t.y();
    cloud.points[i].z = t.z();
  }
*/





 // listener.lookupTransform("/odom_combined", "/base_link", ros::Time(0), transform);


 // listener.transformPointCloud("/odom_combined", snap_res.cloud, cloud);

/*
  offset_.x() = transform.getOrigin().x();
  offset_.y() = transform.getOrigin().y();
  offset_.z() = transform.getOrigin().z();

  rotation_.x() = transform.getRotation().x();
  rotation_.y() = transform.getRotation().y();
  rotation_.z() = transform.getRotation().z();
  rotation_.w() = transform.getRotation().w();
*/



 // cloud = snap_res.cloud;

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
  pcl::fromROSMsg(cloud2, pcl_cloud);

 // pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
 // pcl::transformPointCloud(pcl_cloud, transformed_cloud, newo, rotation_ );



  //pcl::io::savePCDFileASCII ("test_pcd.pcd", pcl_cloud);
 // pcl::io::savePCDFileASCII (boost::lexical_cast<std::string>(transformed_cloud.header.stamp) , transformed_cloud);

 // ROS_INFO ("Saved %d data points to test_pcd.pcd.", (int)transformed_cloud.points.size ());


//  ROS_INFO("rp: %f %f %f", offset_.x(), offset_.y(), offset_.z());



  //  pcl::io::savePCDFileASCII ("test.pcd" , pcl_cloud);







  /*
  // transform point cloud into '/world' frame
  sensor_msgs::PointCloud tmp_cloud;
  tmp_cloud.header = cloud.header;
  //tmp_cloud.header.frame_id = "base_link";
  tmp_cloud.points.resize(cloud.points.size());

  tf::TransformListener listener(ros::Duration(10));

  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";
  laser_point.header.stamp = ros::Time();

  geometry_msgs::PointStamped point;
 // point.header.frame_id = "world";
 // point.header.stamp = ros::Time();



  for(int i=0; i<(int)cloud.points.size(); i++)
  {
    laser_point.point.x = cloud.points[i].x;
    laser_point.point.y = cloud.points[i].y;
    laser_point.point.z = cloud.points[i].z;


    listener.transformPoint("base_link", laser_point, point);



    tmp_cloud.points[i].x = point.point.x;
    tmp_cloud.points[i].y = point.point.y;
    tmp_cloud.points[i].z = point.point.z;


    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) ",
            laser_point.point.x, laser_point.point.y, laser_point.point.z,
            point.point.x, point.point.y, point.point.z);






  }

*/
 // ROS_INFO("DONE");

//  cloud = tmp_cloud;

  return true;

}

bool PointCloud::stopTilt()
{
  pr2_msgs::SetPeriodicCmd::Request scan_req;
  pr2_msgs::SetPeriodicCmd::Response scan_res;

  float min_ang = -0.785;
  float max_ang = 0.9;

  scan_req.command.amplitude  = fabs(min_ang - max_ang) / 2.0;
  scan_req.command.offset = (min_ang + max_ang) / 2.0;
  scan_req.command.period = 3.0; //2.3125;
  scan_req.command.profile = "linear";


  if (!ros::service::call("laser_tilt_controller/set_periodic_cmd", scan_req, scan_res)){
    ROS_ERROR("TiltLaser: error setting laser scanner periodic command");
    return false;
  }else{
    ROS_INFO("TiltLaser : commanded tilt laser scanner with period %f, amplitude %f and offset %f",
             scan_req.command.period, scan_req.command.amplitude, scan_req.command.offset);
    return true;
  }
}



int main(int argc, char** argv)
{

  ros::init(argc, argv, "point_cloud");
  ros::NodeHandle n;

  PointCloud pointcloud(n);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  //ros::spin();

  return 0;
}
