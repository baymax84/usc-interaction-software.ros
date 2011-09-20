/*
 * main_control.cpp
 *
 *  Created on: Dec 13, 2010
 *      Author: Christian Potthast
 */

#include <nbv_arm_planning/main_control.h>

MainControl::MainControl(ros::NodeHandle& n)
{
  node_handle_ = n;
  num_ = 0;
  status_ = 1;

  timer_ = node_handle_.createTimer(ros::Duration(1,0), &MainControl::timerCallback, this);

  // subscribe to the robot pose topic
  //robot_pose_sub_ = node_handle_.subscribe("/robot_pose_ekf/odom_combined",
  //                                         1, &MainControl::robotPoseCallback, this);

/*
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
*/

  ROS_INFO("TEST");


}

void MainControl::timerCallback(const ros::TimerEvent& event)
{

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
 //   getCloud();
 //   saveCloud();
 //   ROS_INFO("GET CLOUD DONE");
    getGoalPoseArm();
    ROS_INFO("getGoalPoseArm DONE");

    ROS_INFO("goal pose: %f %f ", goal_pose_.pose.position.x, goal_pose_.pose.position.y);

    ROS_INFO("status: %d", status_);

    if(status_ == 1)
    {
 //     sendArmGoalCmd();
 //     ROS_INFO("sendArmGoalCmd DONE");

      //sendGoalCmd();
      //ROS_INFO("sendGoalCmd DONE");
      //status_ = 3;
    }

//    savePos();
//    ROS_INFO("save DONE");

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
  nbv_utilities::BoolReturn::Request req;
  nbv_utilities::BoolReturn::Response res;

  req.in = true;

  if (!ros::service::call("nbv_utilities/DriveToInitPosSrv", req, res)){
    ROS_ERROR("MainControl could not start DriveToInitPos");
  }else{
    ROS_INFO("MainControl : Robot at initial position");
  }

  status_ = 1;

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

bool MainControl::getGoalPose()
{
  nbv_arm_planning::SendCloud::Request req;
  nbv_arm_planning::SendCloud::Response res;

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

bool MainControl::getGoalPoseArm()
{
  ROS_INFO("TEST2");

  sensor_msgs::PointCloud2 cloud_;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_;
  Eigen::Vector4f origin_;
  Eigen::Quaternionf orientation_;

  ROS_INFO("TEST3");

  std::string tf_frame_  = ("/base_link");
  std::string file_name = "/home/potthast/projects/data/kinect/init.pcd";

  if (file_name == "" || pcl::io::loadPCDFile (file_name, cloud_, origin_, orientation_) == -1){
    ROS_ERROR("FILE PROBLEM");
    return (-1);
  }

  ROS_INFO ("Loaded a point cloud with %d points (total size is %d) and the following channels: %s.",
                cloud_.width * cloud_.height,
                (uint) cloud_.data.size (), pcl::getFieldsList (cloud_).c_str ());


  ROS_INFO("TEST4");

  pcl::fromROSMsg (cloud_, pcl_cloud_);
  cloud_.header.stamp = ros::Time();
  cloud_.header.frame_id = tf_frame_;

  nbv_arm_planning::SendCloud::Request req;
  nbv_arm_planning::SendCloud::Response res;

  ROS_INFO("TEST5");

  req.cloud = cloud_;
  req.robot_pose = robot_pose_;
  // set the z coordinate to the height of the laser
  req.robot_pose.pose.pose.position.z = transform_.getOrigin().z();

  ROS_INFO("TEST6");

  if (!ros::service::call("extractObjects/GetArmPoseGoal", req, res)){
    ROS_ERROR("MainControl: could not receive robot goal pose from nbv_main/getGoalPoseArm");
    return false;

  }else{
    ROS_INFO("MainControl : Received robot arm goal pose");
  }

  ROS_INFO("TEST7");

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
  pcl::io::savePCDFileASCII (ss , pcl_cloud);
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

void MainControl::sendArmGoalCmd()
{
  nbv_utilities::ArmPoseGoal::Request req;
  nbv_utilities::ArmPoseGoal::Response res;
  move_arm_msgs::MoveArmGoal goal;
  motion_planning_msgs::SimplePoseConstraint desired_pose;

  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  req.arm_pose_goal = goal;

  desired_pose.header.frame_id = "torso_lift_link";
  desired_pose.link_name = "r_wrist_roll_link";
 // desired_pose.pose.position = goal_pose_.pose.position;
 // desired_pose.pose.orientation = goal_pose_.pose.orientation;

  desired_pose.pose.position.x = 0.75;
  desired_pose.pose.position.y = -0.188;
  desired_pose.pose.position.z = goal_pose_.pose.position.z;

  desired_pose.pose.orientation.x = 0.0;
  desired_pose.pose.orientation.y = 0.0;
  desired_pose.pose.orientation.z = 0.0;
  desired_pose.pose.orientation.w = 1.0;



  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.02;

  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;

  req.simple_pose_constraint = desired_pose;

  if (!ros::service::call("nbv_utilities/ArmPoseGoal", req, res)){
    ROS_ERROR("MainControl: Error from nbv_utilities/ArmPoseGoal");
  }else{
    ROS_INFO("MainControl : Arm at position");
  }

  if(res.result == res.FAILED)
    ROS_ERROR("MainControl: Error from nbv_utilities/ArmPoseGoal - SimpleClientGoalState");


}





int main(int argc, char** argv)
{

  ros::init(argc, argv, "main_control");
  ros::NodeHandle n;

  MainControl mainControl(n);

//mainControl.getGoalPoseArm();
  //mainControl.driveToInitPos();
  mainControl.setStatus(1);

  ros::spin();


  return 0;
}
