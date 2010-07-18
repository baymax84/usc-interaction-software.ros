//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#include <bandit_ik/bandit_arm_ik_nl_jr_solver.h>

using namespace Eigen;
using namespace bandit_ik;

BanditArmIKSolver::BanditArmIKSolver(const urdf::Model &robot_model, 
															 const KDL::Chain kdl_chain,
                               const std::string &root_frame_name,
                               const std::string &tip_frame_name,
                               const double &search_discretization_angle, 
                               const int &free_angle):ChainIkSolverPos()
{
  search_discretization_angle_ = search_discretization_angle;
  free_angle_ = free_angle;
  root_frame_name_ = root_frame_name;
  if(!init(robot_model,kdl_chain,root_frame_name,tip_frame_name))
    active_ = false;
  else
    active_ = true;
}

bool BanditArmIKSolver::init(const urdf::Model &robot_model, const KDL::Chain kdl_chain, const std::string &root_name, const std::string &tip_name)
{

	// allocate new vars (TOOD: get size automatically)
	int num_joints = 0;
  q_min_.resize(6), q_max_.resize(6);

	// copy kdl_chain
	kdl_chain_ = kdl_chain;
	
	// get joint maxs and mins
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
  while(link && num_joints < 6)
  {
    boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(link->parent_joint->name);
		ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
    if(!joint)
    {
      ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
      return false;
    }
    if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
	    if(joint->type != urdf::Joint::CONTINUOUS)
  	  {
				q_min_.data[6-num_joints-1] = joint->safety->soft_lower_limit;
				q_max_.data[6-num_joints-1] = joint->safety->soft_upper_limit;
    	}

			num_joints++;
    	addJointToChainInfo(link->parent_joint,solver_info_);
		}

    link = robot_model.getLink(link->getParent()->name);
	}

  solver_info_.link_names.push_back(tip_name);
  std::reverse(solver_info_.limits.begin(),solver_info_.limits.end());
  std::reverse(solver_info_.joint_names.begin(),solver_info_.joint_names.end());
  std::reverse(solver_info_.link_names.begin(),solver_info_.link_names.end());

	//torso_shoulder_offset_x_ = link_offset[0].position.x;
  //torso_shoulder_offset_y_ = link_offset[0].position.y;
  //torso_shoulder_offset_z_ = link_offset[0].position.z;

	fk_solver_chain_ = new KDL::ChainFkSolverPos_recursive(kdl_chain_);
  ik_solver_vel_ = new KDL::ChainIkSolverVel_pinv(kdl_chain_);
	ik_solver_pos = new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, q_min_, q_max_,
																									*fk_solver_chain_,*ik_solver_vel_,
																									100, 1e-6);

	return true;
}

void BanditArmIKSolver::addJointToChainInfo(boost::shared_ptr<const urdf::Joint> joint, kinematics_msgs::KinematicSolverInfo &info)
{
  motion_planning_msgs::JointLimits limit;
  info.joint_names.push_back(joint->name);//Joints are coming in reverse order
  limit.min_position = joint->safety->soft_lower_limit;
  limit.max_position = joint->safety->soft_upper_limit;
  if(joint->type != urdf::Joint::CONTINUOUS)
  {
    limit.min_position = joint->safety->soft_lower_limit;
    limit.max_position = joint->safety->soft_upper_limit;
    limit.has_position_limits = true;
  }
  else
  {
    limit.min_position = -M_PI;
    limit.max_position = M_PI;
    limit.has_position_limits = false;
  }
  limit.max_velocity = joint->limits->velocity;
  limit.has_velocity_limits = 1;
  info.limits.push_back(limit);
}


void BanditArmIKSolver::getSolverInfo(kinematics_msgs::KinematicSolverInfo &response)
{
	response = solver_info_;
  //bandit_arm_ik_.getSolverInfo(response);
}


int BanditArmIKSolver::CartToJnt(const KDL::JntArray& q_init, 
                              const KDL::Frame& p_in, 
                              KDL::JntArray &q_out)
{
	ROS_INFO( "cartToJnt: %d", free_angle_ );
/*
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  //if(free_angle_ == 7)
  {
    //ROS_INFO("Solving with %f",q_init(0)); 
    //bandit_arm_ik_.computeIKShoulderPan(b,q_init(0));
  }
  //else
  {
    bandit_arm_ik_.computeIKShoulderRoll(b,q_init(2));
  }
  
  if(bandit_arm_ik_.solution_ik_.empty())
	{
		ROS_INFO( "ik empty" );
    return -1;
	}
  double min_distance = 1e6;
  int min_index = -1;

  for(int i=0; i< (int) bandit_arm_ik_.solution_ik_.size(); i++)
  {     
    ROS_DEBUG("Solution : %d",(int)bandit_arm_ik_.solution_ik_.size());

    for(int j=0; j < (int)bandit_arm_ik_.solution_ik_[i].size(); j++)
    {   
      ROS_DEBUG("%d: %f",j,bandit_arm_ik_.solution_ik_[i][j]);
    }
    ROS_DEBUG(" ");
    ROS_DEBUG(" ");

    double tmp_distance = computeEuclideanDistance(bandit_arm_ik_.solution_ik_[i],q_init);
    if(tmp_distance < min_distance)
    {
      min_distance = tmp_distance;
      min_index = i;
    }
  }

	ROS_INFO( "done with cartToJnt" );

  if(min_index > -1)
  {
    q_out.resize((int)bandit_arm_ik_.solution_ik_[min_index].size());
    for(int i=0; i < (int)bandit_arm_ik_.solution_ik_[min_index].size(); i++)
    {   
      q_out(i) = bandit_arm_ik_.solution_ik_[min_index][i];
    }
    return 1;
  }
  else
*/
    return -1;
}

int BanditArmIKSolver::CartToJnt(const KDL::JntArray& q_init, 
                              const KDL::Frame& p_in, 
                              std::vector<KDL::JntArray> &q_out)
{
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  KDL::JntArray q;
	ROS_INFO( "solving ik" );
/*
  //if(free_angle_ == 0)
  //{
	bandit_arm_ik_.computeIKShoulderPan(b,q_init(0));
  //}
  //else
  //{
    //bandit_arm_ik_.computeIKShoulderRoll(b,q_init(0));
  //}
  
  if(bandit_arm_ik_.solution_ik_.empty())
	{
		ROS_WARN( "no ik solution found" );
    return -1;
	}
  q.resize(6);
  q_out.clear();
  for(int i=0; i< (int) bandit_arm_ik_.solution_ik_.size(); i++)
  {     
    for(int j=0; j < 6; j++)
    {   
      q(j) = bandit_arm_ik_.solution_ik_[i][j];
    }
    q_out.push_back(q);
  }
	
	ROS_INFO( "done solving ik" );
*/
  return 1;
}

bool BanditArmIKSolver::getCount(int &count, 
                              const int &max_count, 
                              const int &min_count)
{
  if(count > 0)
  {
    if(-count >= min_count)
    {   
      count = -count;
      return true;
    }
    else if(count+1 <= max_count)
    {
      count = count+1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(1-count <= max_count)
    {
      count = 1-count;
      return true;
    }
    else if(count-1 >= min_count)
    {
      count = count -1;
      return true;
    }
    else
      return false;
  }
}



int BanditArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in, 
                                    const KDL::Frame& p_in, 
                                    std::vector<KDL::JntArray> &q_out, 
                                    const double &timeout)
{
  KDL::JntArray q_init = q_in;
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
/*
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((bandit_arm_ik_.solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-bandit_arm_ik_.solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,bandit_arm_ik_.solver_info_.limits[free_angle_].max_position,bandit_arm_ik_.solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
      return 1;
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_INFO("IK Timed out in %f seconds",timeout);
    return TIMED_OUT;
  }
  else
  {
    ROS_INFO("No IK solution was found");
    return NO_IK_SOLUTION;
  }
*/
  return NO_IK_SOLUTION;

}



int BanditArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in, 
                                    const KDL::Frame& p_in, 
                                    KDL::JntArray &q_out, 
                                    const double &timeout)
{
	ROS_INFO( "BanditArmIKSolver::CartToJntSearch" );
  KDL::JntArray q_init = q_in;
	q_out.resize(6);

	ROS_INFO( "going to: %0.4f %0.4f %0.4f", p_in.p.x(), p_in.p.y(), p_in.p.z() );

	for( int i = 0; i < 6; i++ )
	{
		ROS_INFO( "start_pose(%d): %0.4f", i, q_init.data[i] );
	}

	if( ik_solver_pos->CartToJnt(q_init, p_in, q_out ) < 0 )
	{
		ROS_ERROR( "ik solver fail" );
    return NO_IK_SOLUTION;
	}
/*
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((bandit_arm_ik_.solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-bandit_arm_ik_.solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,bandit_arm_ik_.solver_info_.limits[free_angle_].max_position,bandit_arm_ik_.solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
      return 1;
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_INFO("IK Timed out in %f seconds",timeout);
    return TIMED_OUT;
  }
  else
  {
    ROS_INFO("No IK solution was found");
    return NO_IK_SOLUTION;
  }
*/
  return 1;
}

int BanditArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in, 
                                    const KDL::Frame& p_in, 
                                    KDL::JntArray &q_out, 
                                    const double &timeout, 
                                    motion_planning_msgs::ArmNavigationErrorCodes &error_code,
                                    const boost::function<void(const KDL::JntArray&,const KDL::Frame&,motion_planning_msgs::ArmNavigationErrorCodes &)> &desired_pose_callback,
                                    const boost::function<void(const KDL::JntArray&,const KDL::Frame&,motion_planning_msgs::ArmNavigationErrorCodes &)> &solution_callback)
{
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  KDL::JntArray q_init = q_in;
/*
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((bandit_arm_ik_.solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-bandit_arm_ik_.solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,bandit_arm_ik_.solver_info_.limits[free_angle_].max_position,bandit_arm_ik_.solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);

  if(!desired_pose_callback.empty())
    desired_pose_callback(q_init,p_in,error_code);
  if(error_code.val != error_code.SUCCESS)
  {
    return -1;
  }
  bool callback_check = true;
  if(solution_callback.empty())
    callback_check = false;

  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
    {
      if(callback_check)
      {
        solution_callback(q_out,p_in,error_code);
        if(error_code.val == error_code.SUCCESS)
        {
          return 1;
        }
      }
      else
      {
        error_code.val = error_code.SUCCESS;
        return 1;
      }
    }
    if(!getCount(count,num_positive_increments,-num_negative_increments))
    {
      error_code.val = error_code.NO_IK_SOLUTION;
      return -1;
    }
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_INFO("Redundancy search, index:%d, free angle value: %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_INFO("IK Timed out in %f seconds",timeout);
    error_code.val = error_code.TIMED_OUT;
  }
  else
  {
    ROS_INFO("No IK solution was found");
    error_code.val = error_code.NO_IK_SOLUTION;
  }
*/
  return -1;
}


std::string BanditArmIKSolver::getFrameId()
{
  return root_frame_name_;
}
