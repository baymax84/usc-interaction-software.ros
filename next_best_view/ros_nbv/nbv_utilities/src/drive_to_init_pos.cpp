/*
 * driveToInitPos.cpp
 *
 *  Created on: Dec 6, 2010
 *      Author: Christian Potthast
 */

#include <nbv_utilities/drive_to_init_pos.h>

DriveToInitPos::DriveToInitPos(ros::NodeHandle& n)
{

  node_handle_ = n;
  received_msg_ = false;
  check_ = false;
  kinect_check = true;

  drive_to_init_pos_srv_ = node_handle_.advertiseService("nbv_utilities/DriveToInitPosSrv",
                                                         &DriveToInitPos::driveToInitPosSrv, this);

  // subscribe to the robot pose topic
  robot_pose_sub_ = node_handle_.subscribe("/robot_pose_ekf/odom_combined",
                                           1, &DriveToInitPos::robotPoseCallback, this);


  bool check = false;
  // get the laser joint position
  tf::TransformListener listener;
  while(!check){
    if (listener.waitForTransform("/map", "/laser_tilt_mount_link", ros::Time(0), ros::Duration(0.01)))
    {
      listener.lookupTransform("/map", "/laser_tilt_mount_link", ros::Time(0), transform_);
    }
    if(transform_.getOrigin().z() > 0.5)
      check = true;
  }


}

bool DriveToInitPos::driveToInitPosSrv(nbv_utilities::BoolReturn::Request &req,
                                       nbv_utilities::BoolReturn::Response &res)
{
  ros::Rate loop_rate(5);

  if(req.in == true)
  {
    //while(!received_msg_){
    //  ros::spinOnce();
    //}


    //getCloud();

    /*

    while(!kinect_check)
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
*/

    getKinectCloud();

    saveCloud();
    getGoalPose();
    sendGoalCmd();

    //savePos();

    res.out = true;


   // ros::shutdown();

  }

  check_ = true;

  return true;
}


bool DriveToInitPos::getKinectCloud()
{
  /*
  kinect_check = false;

  sub_ = node_handle_.subscribe ("/camera/rgb/points", 1, &DriveToInitPos::getPointCloud, this);
*/

  nbv_utilities::PcdFile::Request req_pcd;
  nbv_utilities::PcdFile::Response res_pcd;
  req_pcd.req = true;

  if (!ros::service::call("nbv_utilities/kinect_grab_one_pc_srv", req_pcd, res_pcd)){
    ROS_ERROR("DriveToInitPos could not receive cloud from nbv_utilities/kinect_grab_one_pc_srv");
    return false;

  }else{
    ROS_INFO("DriveToInitPos : received cloud with size: %d",
             (int)(res_pcd.cloud.height * res_pcd.cloud.width));

  }
  cloud_ = res_pcd.cloud;


  return true;
}

void
DriveToInitPos::getPointCloud (const sensor_msgs::PointCloud2 &msg)
{
  //ROS_INFO("test-1dddd");

  //printf ("PointXYZRGB\n");
  //printf ("Cloud: width = %d, height = %d\n", msg.width, msg.height);

  //ROS_INFO("Cloud: width = %d, height = %d\n", msg.width, msg.height);


  cloud_ = msg;
  transformCloud();

//  PointCloud cloud = *msg;
//  pcl::toROSMsg(cloud, cloud_);

  if(cloud_.data.size() > 1)
  {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg (cloud_, pcl_cloud);
    pcl_ros::transformPointCloud (pcl_cloud, pcl_cloud, transform_kinect_);
    pcl_cloud.header.frame_id = "/map";
    pcl::toROSMsg(pcl_cloud, cloud_);

    kinect_check = true;

    sub_.shutdown();
  }

}

void
DriveToInitPos::transformCloud ()
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  bool check = false;
  int c=0;
  while(!check){
    c++;
    if (listener.waitForTransform("/map", cloud_.header.frame_id, ros::Time(0) ,ros::Duration(5)))
    {
      ros::Duration(1.0).sleep();
      listener.lookupTransform( "/map", cloud_.header.frame_id, ros::Time(0), transform);
    }

 //   ROS_INFO("%f %f %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

   // if(transform_kinect_.getOrigin().z() > 0.001)
   //   check = true;
    if(c > 5)
      check = true;
  }

  transform_kinect_ = transform;


}



void DriveToInitPos::robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  robot_pose_ = msg;

  received_msg_ = true;
}

bool DriveToInitPos::getCloud()
{
  nbv_utilities::PcdFile::Request req;
  nbv_utilities::PcdFile::Response res;

  req.req = true;

  if (!ros::service::call("nbv_utilities/SendCloud", req, res)){
    ROS_ERROR("DriveToInitPos could not receive cloud from nbv_utilities/SendCloud");
    return false;

  }else{
    ROS_INFO("DriveToInitPos : received cloud with size: %d",
             (int)(res.cloud.height * res.cloud.width));

  }

//  cloud_ = res.cloud;

  return true;
}

bool DriveToInitPos::getGoalPose()
{
  nbv_utilities::FindTable::Request req;
  nbv_utilities::FindTable::Response res;

  req.cloud = cloud_;
  req.robot_pose = robot_pose_;
  // set the z coordinate to the height of the laser
  req.robot_pose.pose.pose.position.z = transform_.getOrigin().z();

  if (!ros::service::call("extractObjects/FindTable", req, res)){
    ROS_ERROR("DriveToInitPos: error could not call nbv_main/FindTable");
    return false;

  }else{
    ROS_INFO("DriveToInitPos : received robot goal pose");
  }

  goal_pose_ = res.goal_pose;

  return true;
}


void DriveToInitPos::sendGoalCmd()
{
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
 while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goal_pose_.pose.position.x;
  goal.target_pose.pose.position.y = goal_pose_.pose.position.y;
//  goal.target_pose.pose.position.z = goalPos.z();

  goal.target_pose.pose.orientation = goal_pose_.pose.orientation;


  //ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Goal reached");

  }else
    ROS_INFO("Cloudn't reach the goal !!!!!");

}

void DriveToInitPos::saveCloud()
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(cloud_, pcl_cloud);

  //set the sensor origin (robot pose)
  pcl_cloud.sensor_origin_.x() = robot_pose_.pose.pose.position.x;
  pcl_cloud.sensor_origin_.y() = robot_pose_.pose.pose.position.y;
  pcl_cloud.sensor_origin_.z() = transform_.getOrigin().z();
  pcl_cloud.sensor_orientation_.x() = robot_pose_.pose.pose.orientation.x;
  pcl_cloud.sensor_orientation_.y() = robot_pose_.pose.pose.orientation.y;
  pcl_cloud.sensor_orientation_.z() = robot_pose_.pose.pose.orientation.z;
  pcl_cloud.sensor_orientation_.w() = robot_pose_.pose.pose.orientation.w;

  pcl::io::savePCDFileASCII ("/home/potthast/projects/data/init.pcd" , pcl_cloud);
}

void DriveToInitPos::savePos()
{
  std::ofstream SaveFile("/home/potthast/projects/data/init_poses.txt");
  SaveFile << robot_pose_.pose.pose.position.x << " " << robot_pose_.pose.pose.position.y <<
      " " << transform_.getOrigin().z();
  SaveFile.close();
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "nbv_utilities_drive_to_init_pos");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);

  DriveToInitPos drive_to_init_pos(n);


  while(!drive_to_init_pos.check_){
    ros::spinOnce();

    loop_rate.sleep();
  }

  ros::shutdown();


  return 0;
}
