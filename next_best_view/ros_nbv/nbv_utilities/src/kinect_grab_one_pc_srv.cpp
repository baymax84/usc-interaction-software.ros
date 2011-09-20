/*
 * kinect_grab_one_pc_srv.cpp
 *
 *  Created on: Mar 17, 2011
 *      Author: potthast
 */

#include "nbv_utilities/kinect_grab_one_pc_srv.h"

KinectGrabOnePCSrv::KinectGrabOnePCSrv (ros::NodeHandle& n)
{

  frame_id_ = "/map";
  node_handle_ = n;
  kinect_grab_one_pc_srv_ = node_handle_.advertiseService ("nbv_utilities/kinect_grab_one_pc_srv",
                                                           &KinectGrabOnePCSrv::kinect_grab_one_pc, this);

  //ROS_INFO("%f %f %f", kinect_pose_.pose.pose.orientation.x, kinect_pose_.pose.pose.orientation.y, kinect_pose_.pose.pose.orientation.z);



}

/*
 void KinectGrabOnePCSrv::kinectPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
 {
 kinect_pose_ = msg;

 ROS_INFO("%f %f %f", kinect_pose_.pose.pose.position.x, kinect_pose_.pose.pose.position.y, kinect_pose_.pose.pose.position.z);

 }
 */


bool
KinectGrabOnePCSrv::getKinectCloud()
{

  kinect_check = false;
  //ros::Duration(1.0).sleep();
  tf::StampedTransform transform;


  tf::TransformListener listener;
  bool check = false;
  while (!check)
  {
    // get the kinect frame pose

    if (listener.waitForTransform (frame_id_, "/openni_rgb_optical_frame", ros::Time (0), ros::Duration (0.01)))
    {
      //ros::Duration(1.0).sleep();
      //ROS_INFO("%f %f %f", transform_.getOrigin().x(), transform_.getOrigin().y(), transform_.getOrigin().z());
      listener.lookupTransform (frame_id_, "/openni_rgb_optical_frame", ros::Time (0), transform);
    }

    ROS_INFO("%f %f %f", transform_.getOrigin().x(), transform_.getOrigin().y(), transform_.getOrigin().z());

    if (transform.getOrigin ().z () > 0.1)
      check = true;
  }

  kinect_pose_.pose.pose.position.x = transform.getOrigin ().x ();
  kinect_pose_.pose.pose.position.y = transform.getOrigin ().y ();
  kinect_pose_.pose.pose.position.z = transform.getOrigin ().z ();

  kinect_pose_.pose.pose.orientation.x = (-1) * transform.getRotation ().x ();
  kinect_pose_.pose.pose.orientation.y = (-1) * transform.getRotation ().y ();
  kinect_pose_.pose.pose.orientation.z = (-1) * transform.getRotation ().z ();
  kinect_pose_.pose.pose.orientation.w = (-1) * transform.getRotation ().w ();

  transform_ = transform;

  sub_ = node_handle_.subscribe ("/camera/rgb/points", 1, &KinectGrabOnePCSrv::getPointCloud, this);
  ros::Duration(1.0).sleep();

  return true;
}


bool
KinectGrabOnePCSrv::kinect_grab_one_pc (nbv_utilities::PcdFile::Request &req, nbv_utilities::PcdFile::Response &res)
{

  ros::Rate loop_rate(5);

  if(req.req == true)
  {
    getKinectCloud();
    while(!kinect_check)
    {
      ros::spinOnce();
      loop_rate.sleep();
    }




  }

  res.cloud = point_cloud_;

  return true;
}

void
KinectGrabOnePCSrv::transformCloud ()
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  bool check = false;
  int c=0;
  while(!check){
    c++;
    if (listener.waitForTransform(frame_id_, point_cloud_.header.frame_id, ros::Time(0) ,ros::Duration(5)))
    {
      ros::Duration(1.0).sleep();
      listener.lookupTransform(frame_id_, point_cloud_.header.frame_id, ros::Time(0), transform);
    }

    ROS_INFO("%f %f %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

    //if(transform.getOrigin().x() > 0.001)
    //  check = true;
    if(c > 5)
      check = true;
  }

  transform_kinect_ = transform;

}

void
KinectGrabOnePCSrv::saveCloud ()
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg (point_cloud_, pcl_cloud);

  pcl_ros::transformPointCloud (pcl_cloud, pcl_cloud, transform_kinect_);

  pcl_cloud.sensor_origin_.x () = kinect_pose_.pose.pose.position.x;
  pcl_cloud.sensor_origin_.y () = kinect_pose_.pose.pose.position.y;
  pcl_cloud.sensor_origin_.z () = kinect_pose_.pose.pose.position.z;
  pcl_cloud.sensor_origin_.w () = 0.0;
  pcl_cloud.sensor_orientation_.x () = kinect_pose_.pose.pose.orientation.x;
  pcl_cloud.sensor_orientation_.y () = kinect_pose_.pose.pose.orientation.y;
  pcl_cloud.sensor_orientation_.z () = kinect_pose_.pose.pose.orientation.z;
  pcl_cloud.sensor_orientation_.w () = kinect_pose_.pose.pose.orientation.w;

  pcl::io::savePCDFileASCII ("/home/potthast/projects/data/init.pcd", pcl_cloud);
}

void
KinectGrabOnePCSrv::getPointCloud (const sensor_msgs::PointCloud2 &msg)
{
  //ROS_INFO("test-1dddd");

  //printf ("PointXYZRGB\n");
  //printf ("Cloud: width = %d, height = %d\n", msg.width, msg.height);

  //ROS_INFO("Cloud: width = %d, height = %d\n", msg.width, msg.height);


  point_cloud_ = msg;
  //ros::Duration(1.0).sleep();


//  PointCloud cloud = *msg;
//  pcl::toROSMsg(cloud, cloud_);

  if(point_cloud_.data.size() > 1)
  {
    transformCloud();

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg (point_cloud_, pcl_cloud);
    pcl_ros::transformPointCloud (pcl_cloud, pcl_cloud, transform_kinect_);
    pcl_cloud.header.frame_id = frame_id_;
    pcl::toROSMsg(pcl_cloud, point_cloud_);

    kinect_check = true;

    sub_.shutdown();
  }
}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "kinect_grab_one_pc_srv");
  ros::NodeHandle n;
  KinectGrabOnePCSrv kinect_grab_one_pc_srv (n);
  ros::Rate loop_rate(5);

  while(n.ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }


  //ros::MultiThreadedSpinner spinner (2);
  //spinner.spin ();

  return 0;
}
