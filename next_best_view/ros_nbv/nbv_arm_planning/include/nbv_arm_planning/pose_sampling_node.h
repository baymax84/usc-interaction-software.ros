/*
 * pose_sampling_node.h
 *
 *  Created on: Apr 21, 2011
 *      Author: potthast
 */

#ifndef POSE_SAMPLING_NODE_H_
#define POSE_SAMPLING_NODE_H_

#include <ros/ros.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nbv_arm_planning/SamplingPoses.h>

//#include <boost/random/mersenne_twister.hpp>
//#include <boost/random/uniform_int_distribution.hpp>

#include <boost/random.hpp>


#include <pcl/sample_consensus/sac_model_normal_plane.h>







class PoseSamplingNode
{
public:
  PoseSamplingNode (ros::NodeHandle& n);

  bool
  PoseSamplingNodeCallback (nbv_arm_planning::SamplingPoses::Request &req,
                            nbv_arm_planning::SamplingPoses::Response &res);

  void
  computeSamplingPoses ();
  bool
  ik_solver (geometry_msgs::PoseStamped& pose);
  Eigen::Vector3f
  samplePose ();
  double
  randomNumber (float high, float low);

  float eulcideanDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);

private:
  ros::NodeHandle node_handle_;
  ros::ServiceServer pose_sampling_node_srv_;

  geometry_msgs::PoseArray sampling_poses_;

  // ik service
  ros::ServiceClient query_client_;
  ros::ServiceClient ik_client_;
  kinematics_msgs::GetKinematicSolverInfo::Request ik_req_;
  kinematics_msgs::GetKinematicSolverInfo::Response ik_res_;

  boost::mt19937 gen_;

};

#endif /* POSE_SAMPLING_NODE_H_ */
