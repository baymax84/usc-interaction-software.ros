/***************************************************************************
 *  include/kinematic_retargeting/humanoid_conversions.h
 *  --------------------
 *
 *  Copyright (c) 2012, Dylan J. Foster
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of kinematic_retargeting nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#ifndef KINEMATICRETARGETING_HUMANOIDCONVERSIONS_H_
#define KINEMATICRETARGETING_HUMANOIDCONVERSIONS_H_


// humanoid 
#include <humanoid/humanoid_features.h>
#include <humanoid/spatial_metrics.h>
#include <humanoid_models/HumanoidJoint.h>


// kdl
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

// quickdev
#include <quickdev/runable_policy.h>

// tf
#include <tf_conversions/tf_kdl.h>

// kinematic retargeting toolkit
#include <rtk/math_impl.h>

using humanoid::_HumanoidStateMsg;

using rtk::_Joint;
using rtk::_Segment;
using rtk::_Frame;
using rtk::_JntArray;
using rtk::_Vector;
using rtk::_Rotation;
using rtk::_Chain;

typedef rtk::_FrameArray _FrameArray;

typedef humanoid::_JointName _JointName;
typedef humanoid::Humanoid _Humanoid;
typedef humanoid_models::HumanoidJoint  _HumanoidJointMsg;

static int forwardKinematics(const _Chain & chain, 
			     const _JntArray & angles, 
			     _HumanoidStateMsg & positions)
{
  if(chain.getNrOfJoints() != angles.rows())
    {
      ROS_WARN("Number of joints (%d) != number of angles (%d)",
	       chain.getNrOfJoints(), angles.rows());
      return -1;
    }
    
  KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);


  _JointName parent_name = "";
  _JointName current_name;

  for(unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
    {	
      geometry_msgs::Pose joint_pose;
      _HumanoidJointMsg joint;
      KDL::Frame joint_frame;
      
      int error = fksolver.JntToCart(angles, joint_frame, i);
      if(error)
	{
	  ROS_WARN("Forward kinematic solver failed");
	  return -1;
	}
	
      tf::PoseKDLToMsg(joint_frame, joint_pose);

      // Print out value of pose here - checks to see whether or not messages are messed up coming out of fksolver

      joint.pose.pose = joint_pose;

      /* Set the names of the joints ------------------------ */
      current_name = chain.getSegment(i).getJoint().getName();
      /* ROS_INFO("in name loop iteration %d, name %s", i, current_name.c_str()); */
      joint.name = current_name;
      joint.parent_name = parent_name;
      parent_name = current_name;
      
      positions.joints.push_back(joint);

    }

  return 0;
}

static double getHumanoidChainLength(_HumanoidStateMsg const & chain)
  {
    double length = 0.0;
    
    for(auto chain_it = ++chain.joints.begin(); chain_it !=chain.joints.end(); ++chain_it)
      {
	length += humanoid::spatial::getDistance(*chain_it, *(chain_it-1)).length();
      }
    return length;
  }

static int normalizeTargetToSourceLength(_HumanoidStateMsg const & source, _HumanoidStateMsg & target)
  {
    double source_length, target_length, length_ratio;

    source_length = getHumanoidChainLength(source);
    target_length = getHumanoidChainLength(target);
    length_ratio = source_length/target_length;
    
    for(auto target_it = ++target.joints.begin(); target_it != target.joints.end(); ++target_it)
      {
	target_it->pose.pose.position.x *= length_ratio;
	target_it->pose.pose.position.y *= length_ratio;
	target_it->pose.pose.position.z *= length_ratio;
      }
    
    return 0;
  }

  int frameArrayFromHumanoid(_JointName const & begin_name,
			     _JointName const & end_name,
			     _JointName const & parent_name,
			     _Humanoid const & humanoid,
			     _FrameArray & extracted_chain)
  {
    _JointName previous_name;
    _HumanoidJointMsg source_joint = humanoid[end_name];
    _HumanoidJointMsg parent_joint = humanoid[parent_name];
    _Frame parent_frame;
    _Frame source_frame;
    
    tf::PoseMsgToKDL(parent_joint.pose.pose, parent_frame);
        
    if(source_joint.name == "")
      {
	ROS_WARN("End effector joint lookup failed. ( %s )", end_name.c_str());
	return -1;
      }

    if(parent_joint.name == "")
      {
	ROS_WARN("Chain parent joint lookup failed. ( %s )", parent_name.c_str());
	return -1;
      }
        
    tf::PoseMsgToKDL(source_joint.pose.pose, source_frame);
    
    extracted_chain.push_back(parent_frame.Inverse() * source_frame);
    
    // Traverse chain from end effector to base joint
    while( source_joint.name != begin_name )
      {
	previous_name = source_joint.name;
	source_joint = *(humanoid.find(source_joint.parent_name));
	
	// Implies that getParentJoint() failed to find a parent 
	if(source_joint.name == "")
	  {
	    ROS_WARN("Joint lookup failed. ( Parent of %s )", previous_name.c_str());
	    return -1;
	  }

	tf::PoseMsgToKDL(source_joint.pose.pose, source_frame);
	
	extracted_chain.push_back(parent_frame.Inverse() * source_frame );
      }
    
    std::reverse(extracted_chain.begin(), extracted_chain.end());
    
    return 0;
  }

#endif // KINEMATICRETARGETING_HUMANOIDCONVERSIONS_H 
