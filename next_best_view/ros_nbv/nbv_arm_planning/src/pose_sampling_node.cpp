/*
 * pose_sampling_node.cpp
 *
 *  Created on: Apr 21, 2011
 *      Author: potthast
 */

#include <nbv_arm_planning/pose_sampling_node.h>

PoseSamplingNode::PoseSamplingNode(ros::NodeHandle& n)
{
  node_handle_ = n;

  pose_sampling_node_srv_ = node_handle_.advertiseService("nbv_arm_planning/PoseSamplingNode",
                                                          &PoseSamplingNode::PoseSamplingNodeCallback,
                                                          this);

 // ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
 // ros::service::waitForService("pr2_right_arm_kinematics/get_ik");

  query_client_ = node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
  ik_client_ = node_handle_.serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");

  query_client_.call(ik_req_,ik_res_);
}

bool PoseSamplingNode::PoseSamplingNodeCallback(nbv_arm_planning::SamplingPoses::Request &req,
                                                nbv_arm_planning::SamplingPoses::Response &res)
{
  boost::mt19937 gen(std::time(NULL));
  gen_ = gen;

  if(req.req == true)
  {
    sampling_poses_.poses.clear();
    int c=0;
    while(sampling_poses_.poses.size() < 40 )
    //for(unsigned int i=0; i<25; i++)
    {
      c++;
      computeSamplingPoses();
    }
  }

  res.poses = sampling_poses_;

  return true;
}

float PoseSamplingNode::eulcideanDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  float dist;

  dist = sqrt( pow(p1.position.x - p2.position.x, 2) +
               pow(p1.position.y - p2.position.y, 2) +
               pow(p1.position.z - p2.position.z, 2) );

  return dist;

}

void PoseSamplingNode::computeSamplingPoses()
{

  geometry_msgs::PoseStamped pose;

  Eigen::Vector3f sampled_pose;
  sampled_pose = samplePose();

  std::vector<Eigen::Vector3f> center;
  Eigen::Vector3f c;
  c.x() =  0.746465;
  c.y() = -0.175320;
  c.z() = 0.775608;
  center.push_back(c);
  c.x() =  0.736526;
  c.y() = 0.226792;
  c.z() = 0.799729;
  center.push_back(c);
  c.x() =  0.731130;
  c.y() = 0.404912;
  c.z() = 0.798192;
  center.push_back(c);


  boost::uniform_int<> gm(1, 3);
  boost::variate_generator<boost::mt19937&, boost::uniform_int<> > vg( gen_, gm );

  int num = vg();

 // ROS_INFO("center: %d", num-1 );
  c = center[num-1];


  c.x() =  randomNumber(1.24, 0.52);
  c.y() = randomNumber(0.64, -0.83);
  c.z() = 0.7;



  Eigen::Vector3f x = c-sampled_pose;
  x.normalize();
  Eigen::Vector3f x_old(1.0,0.0,0.0);
  Eigen::Vector3f y = x_old.cross(x);
  y.normalize();
  Eigen::Vector3f z = x.cross(y);
  z.normalize();

  Eigen::Matrix3f m;
  m << x.x(), y.x(), z.x(),
       x.y(), y.y(), z.y(),
       x.z(), y.z(), z.z();
  //std::cout << m;

  Eigen::Quaternionf quad(m);

  pose.pose.position.x = sampled_pose.x();
  pose.pose.position.y = sampled_pose.y();
  pose.pose.position.z = sampled_pose.z();
  pose.pose.orientation.x = quad.x();
  pose.pose.orientation.y = quad.y();
  pose.pose.orientation.z = quad.z();
  pose.pose.orientation.w = quad.w();


  float dist = 0.0;
  bool insert=true;
  for(unsigned int i=0;i<sampling_poses_.poses.size();i++)
  {

    geometry_msgs::Pose p;
    p.position = sampling_poses_.poses[i].position;
    dist = eulcideanDistance( p, pose.pose );
    if( dist < 0.1 )
    {
      insert=false;
    }

  }

//  pose.pose.orientation.x = 0.0092;
//  pose.pose.orientation.y = -0.2725;
//  pose.pose.orientation.z = 0.15268;
//  pose.pose.orientation.w = -0.94990;

  if(insert)
  {
    ROS_INFO("dist: %f", dist);
    bool ik = ik_solver(pose);
    if( ik )
    {
      geometry_msgs::Pose p;
      p.position = pose.pose.position;
      p.orientation = pose.pose.orientation;

      sampling_poses_.poses.push_back(p);
    }

    sampling_poses_.header.stamp = ros::Time::now();
  }
}

double PoseSamplingNode::randomNumber(float high, float low)
{
  double range = high - low;
  double num = static_cast<double>( rand() ) * range / static_cast<double>( RAND_MAX ) + low ;
  return num;
}

Eigen::Vector3f PoseSamplingNode::samplePose()
{
  Eigen::Vector3f pose;

  pose.x() = randomNumber(0.7, 0.15);
  pose.y() = randomNumber(0.8, -0.7);
  pose.z() = randomNumber(1.4, 1.25);

 // ROS_INFO("%f", num);



  return pose;
}

bool PoseSamplingNode::ik_solver(geometry_msgs::PoseStamped& pose)
{
  kinematics_msgs::GetPositionIK::Request  gpik_req;
  kinematics_msgs::GetPositionIK::Response gpik_res;
  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "r_wrist_roll_link";

  gpik_req.ik_request.pose_stamped.header.frame_id = "base_link";
  gpik_req.ik_request.pose_stamped.pose = pose.pose;

  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(ik_res_.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = ik_res_.kinematic_solver_info.joint_names;
  for(unsigned int i=0; i< ik_res_.kinematic_solver_info.joint_names.size(); i++)
  {
    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (ik_res_.kinematic_solver_info.limits[i].min_position + ik_res_.kinematic_solver_info.limits[i].max_position)/2.0;
  }
  if(ik_client_.call(gpik_req, gpik_res))
  {
    if(!(gpik_res.error_code.val == gpik_res.error_code.SUCCESS))
    {
      ROS_ERROR("Inverse kinematics service call failed");
      return false;
    }
  }

  ROS_INFO("Inverse kinematics service call succeded");
  return true;
}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pose_sampling_node");
  ros::NodeHandle n;

  PoseSamplingNode poseSamplingNode(n);

  //poseSamplingNode.computeSamplingPoses();

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
