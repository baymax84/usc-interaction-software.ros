/*
 * main_control.cpp
 *
 *  Created on: Dec 13, 2010
 *      Author: Christian Potthast
 */

#include <nbv_main/main_control.h>

MainControl::MainControl(ros::NodeHandle& n)
{
  node_handle_ = n;
  num_ = 0;
  status_ = 0;
  kinect_check = true;

  timer_ = node_handle_.createTimer(ros::Duration(1,0), &MainControl::timerCallback, this);

  // subscribe to the robot pose topic
  robot_pose_sub_ = node_handle_.subscribe("/robot_pose_ekf/odom_combined",
                                           1, &MainControl::robotPoseCallback, this);


  tf::TransformListener listener;
  bool check = false;
  while(!check){
    // get the laser joint position

    if (listener.waitForTransform("/map", "/laser_tilt_mount_link", ros::Time(0), ros::Duration(0.01)))
    {
      ROS_INFO("%f %f %f", transform_.getOrigin().x(), transform_.getOrigin().y(), transform_.getOrigin().z());
      listener.lookupTransform("/map", "/laser_tilt_mount_link", ros::Time(0), transform_);
    }

    ROS_INFO("%f %f %f", transform_.getOrigin().x(), transform_.getOrigin().y(), transform_.getOrigin().z());

    if(transform_.getOrigin().z() > 0.5)
      check = true;
  }



}

void MainControl::timerCallback(const ros::TimerEvent& event)
{
  ros::Rate loop_rate(5);

  ROS_INFO("status: %d", status_);

  if(status_ == 2)
  {
    ROS_INFO("DONE");
   // getCloud();
   // getGoalPose();
   // save();
    status_ = 3;

   // ros::shutdown();
  }

  if(status_ == 1)
  {
    getKinectCloud();
    sendKinectCloud();
    /*
    while(!kinect_check)
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    */

    ROS_INFO("GET KINECT CLOUD DONE");
    getCloud();
    saveCloud();
    ROS_INFO("GET CLOUD DONE");
    getGoalPose();
    ROS_INFO("getGoalPose DONE");

    ROS_INFO("goal pose: %f %f ", goal_pose_.pose.position.x, goal_pose_.pose.position.y);

    ROS_INFO("status: %d", status_);

    if(status_ == 1)
    {
      sendGoalCmd();
      ROS_INFO("sendGoalCmd DONE");
      //status_ = 3;
    }

    //savePos();
    ROS_INFO("save DONE");

    if (status_ == 3)
      ROS_INFO("status_ DONE");
  }


}


void MainControl::robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  robot_pose_ = msg;

}

void MainControl::driveToInitPos()
{
  nbv_main::BoolReturn::Request req;
  nbv_main::BoolReturn::Response res;

  req.in = true;

  if (!ros::service::call("nbv_utilities/DriveToInitPosSrv", req, res)){
    ROS_ERROR("MainControl could not start DriveToInitPos");
  }else{
    ROS_INFO("MainControl : Robot at initial position");
    status_ = 1;
  }



}

bool MainControl::getCloud()
{
  nbv_utilities::PcdFile::Request req;
  nbv_utilities::PcdFile::Response res;

  req.req = true;

  if (!ros::service::call("nbv_utilities/SendCloud", req, res)){
    ROS_ERROR("MainControl could not receive cloud from nbv_utilities/SendCloud");
    return false;

  }else{
    ROS_INFO("MainControl : Received cloud with size: %d",
             (int)(res.cloud.height * res.cloud.width));

  }

  cloud_ = res.cloud;

  num_++;

  return true;
}

bool MainControl::getKinectCloud()
{

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
  point_cloud_ = res_pcd.cloud;


  /*
  kinect_check = false;

  sub_ = node_handle_.subscribe ("/camera/rgb/points", 1, &MainControl::getPointCloud, this);
*/
  /*
  while (point_cloud_.data.size () < 1)
  {
    transformCloud ();

    ros::Rate r (1);
    r.sleep ();
  }
  sub_.shutdown ();
*/



  return true;
}

void
MainControl::getPointCloud (const sensor_msgs::PointCloud2 &msg)
{
  point_cloud_ = msg;
  transformCloud();

//  PointCloud cloud = *msg;
//  pcl::toROSMsg(cloud, cloud_);

  if(point_cloud_.data.size() > 1)
  {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg (point_cloud_, pcl_cloud);
    pcl_ros::transformPointCloud (pcl_cloud, pcl_cloud, transform_);
    pcl_cloud.header.frame_id = "/map";
    pcl::toROSMsg(pcl_cloud, point_cloud_);

    kinect_check = true;

    sub_.shutdown();
  }

}

void
MainControl::transformCloud ()
{
  tf::TransformListener listener;
  bool check = false;
  int c=0;
  while(!check){
    c++;
    if (listener.waitForTransform("/map", point_cloud_.header.frame_id, ros::Time(0) ,ros::Duration(5)))
    {
      listener.lookupTransform( "/map", point_cloud_.header.frame_id, ros::Time(0), transform_kinect_);
    }

    if(transform_kinect_.getOrigin().z() > 0.001)
      check = true;
    if(c > 100)
      check = true;
  }



  /*
  tf::TransformListener listener;

  if (listener.waitForTransform ("/map", point_cloud_.header.frame_id, point_cloud_.header.stamp,
                                 ros::Duration (5)))
  {
    listener.lookupTransform ("/map", point_cloud_.header.frame_id, point_cloud_.header.stamp, transform_kinect_);
  }
  */
}

bool MainControl::sendKinectCloud()
{
  nbv_utilities::SendCloud::Request req;
  nbv_utilities::SendCloud::Response res;

  req.cloud = point_cloud_;
  req.dir = -1.0;

  if (!ros::service::call("extractObjects/KinectCloud", req, res)){
    ROS_ERROR("MainControl: could not receive robot goal pose from nbv_main/SendCloud");
    return false;

  }else{
    ROS_INFO("MainControl : Send kinect cloud");
  }

  return true;
}



bool MainControl::getGoalPose()
{
  nbv_utilities::SendCloud::Request req;
  nbv_utilities::SendCloud::Response res;

  req.dir = -1.0;
  req.cloud = cloud_;
  req.robot_pose = robot_pose_;
  // set the z coordinate to the height of the laser
  req.robot_pose.pose.pose.position.z = transform_.getOrigin().z();

  if (!ros::service::call("extractObjects/SendCloud", req, res)){
    ROS_ERROR("MainControl: could not receive robot goal pose from nbv_main/SendCloud");
    return false;

  }else{
    ROS_INFO("MainControl : Received robot goal pose");
  }

  status_ = res.result;
  goal_pose_ = res.goal_pose;

  return true;
}

void MainControl::saveCloud()
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(cloud_, pcl_cloud);
  std::string s = boost::lexical_cast<std::string>( num_ );
  std::string ss = "/home/potthast/projects/data/scan_" + s + ".pcd";

  //set the sensor origin (robot pose)
  pcl_cloud.sensor_origin_.x() = robot_pose_.pose.pose.position.x;
  pcl_cloud.sensor_origin_.y() = robot_pose_.pose.pose.position.y;
  pcl_cloud.sensor_origin_.z() = transform_.getOrigin().z();
  pcl_cloud.sensor_orientation_.x() = robot_pose_.pose.pose.orientation.x;
  pcl_cloud.sensor_orientation_.y() = robot_pose_.pose.pose.orientation.y;
  pcl_cloud.sensor_orientation_.z() = robot_pose_.pose.pose.orientation.z;
  pcl_cloud.sensor_orientation_.w() = robot_pose_.pose.pose.orientation.w;

  pcl::io::savePCDFileASCII (ss , pcl_cloud);


  if(kinect_check)
  {
    // save kinect cloud
    pcl::fromROSMsg(point_cloud_, pcl_cloud);
    s = boost::lexical_cast<std::string>( num_ );
    ss = "/home/potthast/projects/data/kinect_" + s + ".pcd";

    pcl::io::savePCDFileASCII (ss , pcl_cloud);
  }
}

void MainControl::savePos()
{
  std::ifstream ifile("/home/potthast/projects/data/scan_poses.txt");
  std::ofstream file;
  if((bool)ifile)
  {
    //open file
    file.open("/home/potthast/projects/data/scan_poses.txt", std::ios::app);
    file << robot_pose_.pose.pose.position.x << " " << robot_pose_.pose.pose.position.y <<
        " " << transform_.getOrigin().z() << "\n";
  }else{
    //create a new file
    file.open("/home/potthast/projects/data/scan_poses.txt");
    file << robot_pose_.pose.pose.position.x << " " << robot_pose_.pose.pose.position.y <<
        " " << transform_.getOrigin().z() << "\n";
  }
  file.close();

  ROS_INFO("MainControl : Data saved");

}

void MainControl::sendGoalCmd()
{
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goal_pose_.pose.position.x;
  goal.target_pose.pose.position.y = goal_pose_.pose.position.y;
  //goal.target_pose.pose.position.z = goalPos.z();
  goal.target_pose.pose.orientation = goal_pose_.pose.orientation;

  //ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("MainControl: Goal reached");

  }else
    ROS_INFO("Cloudn't reach the goal !!!!!");

}




int main(int argc, char** argv)
{

  ros::init(argc, argv, "main_control");
  ros::NodeHandle n;

  MainControl mainControl(n);

  mainControl.driveToInitPos();

  ros::spin();


  return 0;
}
