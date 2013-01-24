/***************************************************************************
 *  joint_state_retargeter_node.h
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
 *  * Neither the name of rtk nor the names of its
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

#ifndef RSSDEMO_JOINTSTATERETARGETERNODE_H_
#define RSSDEMO_JOINTSTATERETARGETERNODE_H_

#include <quickdev/node.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>

/// data
#include <map>
/* #include <xmlrpcpp/XmlRpc.h> */

/// robot model
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>

/// kinematics
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <rtk/math_impl.h>
#include <rtk/pose_solver_lsq.h>

typedef sensor_msgs::JointState _JointStateMsg;

typedef KDL::Joint _Joint;
typedef KDL::Segment _Segment;
typedef KDL::Vector _Vector;
typedef KDL::Rotation _Rotation;
typedef KDL::Frame _Frame;
typedef KDL::Chain _Chain;
typedef KDL::Tree _Tree;
typedef KDL::JntArray _JntArray;

using rtk::_FrameArray;

typedef std::map<std::string, int> _JointOrderMap;

struct Retargeter
{
  std::string name_;
  _Frame target_to_source_;
  
  _Chain source_chain_;
  _Chain target_chain_;

  _JointOrderMap source_joint_order_map_;
  _JntArray target_limits_upper_;
  _JntArray target_limits_lower_;

  KDL::ChainFkSolverPos_recursive source_fk_;
  rtk::PoseSolverLsq retargeter_;

};

QUICKDEV_DECLARE_NODE( JointStateRetargeter )

QUICKDEV_DECLARE_NODE_CLASS( JointStateRetargeter )
{
 protected:
  /// communications
  ros::MultiPublisher<> multi_pub_;
  ros::MultiSubscriber<> multi_sub_;
  
  /// retargeting functionality
  /// Using deque instead of vector because std::vector requires that its storage type have a default constructor and an assignment operator
  std::deque<Retargeter> retargeters_;
  
 QUICKDEV_DECLARE_NODE_CONSTRUCTOR( JointStateRetargeter )
  {
  }
  
  
  QUICKDEV_SPIN_FIRST()
    {
      QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );
      
      /// Initialize communications
      multi_sub_.addSubscriber( nh_rel, "source_joint_states",
				&JointStateRetargeterNode::sourceJointStateCB, this );
      multi_pub_.addPublishers<_JointStateMsg>( nh_rel, { "retargeted_joint_states" } );
      
      /// Get URDF path
      std::string source_urdf_path, target_urdf_path;
      
      if(!nh_rel.getParam("source_urdf", source_urdf_path))
	{
	  ROS_ERROR("Couldn't retrieve source urdf path from parameter server.");
	  exit(1);
	}
      if(!nh_rel.getParam("target_urdf", target_urdf_path))
	{
	  ROS_ERROR("Couldn't retrieve target urdf path from parameter server.");
	  exit(1);
	}
      
      /// Load the trees
      urdf::Model source_model, target_model;

      if(!source_model.initFile(source_urdf_path))
	{
	  ROS_ERROR("Failed to initialize source URDF ( %s ).", source_urdf_path.c_str());
	  exit(1);
	}
      else if (!target_model.initFile(target_urdf_path))
	{
	  ROS_ERROR("Failed to initialize target URDF ( %s ).", target_urdf_path.c_str());
	  exit(1);
	}
      else
	ROS_INFO("Initialized URDF files succesfully." );

      ROS_INFO("Loading kinematic trees...");
      _Tree source_tree, target_tree;
      
      if (!kdl_parser::treeFromUrdfModel(source_model, source_tree))
	{
	  ROS_ERROR("Failed to extract source tree from URDF model.");
	  exit(1);
	}
      if (!kdl_parser::treeFromUrdfModel(target_model, target_tree))
	{
	  ROS_ERROR("Failed to extract target tree from URDF model.");
	  exit(1);
	}
      
      /// Load chain endpoint data
      XmlRpc::XmlRpcValue chain_info;
      if(!nh_rel.getParam("chain_info", chain_info))
	{
	  ROS_ERROR("Couldn't retreive chain metadata from parameter server.");
	  exit(1);
	}
      
      /// TODO: find a better place to put this
      std::map<int, std::string> joint_type_text_map
	    {
	      {urdf::Joint::UNKNOWN, "unknown"},
	      {urdf::Joint::REVOLUTE, "revolute"},
	      {urdf::Joint::CONTINUOUS, "continuous"},
	      {urdf::Joint::PRISMATIC, "prismatic"},
      	      {urdf::Joint::FLOATING, "floating"},	
	      {urdf::Joint::PLANAR, "planar"},
      	      {urdf::Joint::FIXED, "fixed"}
	    };

      /// Get data for each chain that will be retargeted
      for(auto chain_it = chain_info.begin(); chain_it != chain_info.end(); ++chain_it)
	{
	  /// Chains that we will be using for retargeting
	  _Chain source_chain, target_chain;

	  /// The name of the chain being retargeted
	  std::string chain_name = chain_it->first;
	  XmlRpc::XmlRpcValue & source_endpoints = chain_it->second["source"];
	  XmlRpc::XmlRpcValue & target_endpoints = chain_it->second["target"];
	  XmlRpc::XmlRpcValue & t2s_xml = chain_it->second["target_to_source"];
	  
	  _Frame target_to_source = _Frame(_Rotation::RPY(t2s_xml["roll"], t2s_xml["pitch"], t2s_xml["yaw"]),
					   _Vector(t2s_xml["x"], t2s_xml["y"], t2s_xml["z"]));
	  
	  ROS_INFO_STREAM( chain_name << " endpoints: " );
	  ROS_INFO_STREAM( "source:");
	  ROS_INFO_STREAM( "\t" << source_endpoints["begin"]
			   << " ----> "
			   << source_endpoints["end"]);
	  ROS_INFO_STREAM( "target:" );
	  ROS_INFO_STREAM( target_endpoints["begin"]
			   << " ----> "
			   << target_endpoints["end"] );
	  ROS_INFO("--------------------");
	  
	  if( !source_tree.getChain(source_endpoints["begin"], source_endpoints["end"], source_chain) )
	    {
	      ROS_WARN("Failed to extract source chain for ( %s ). Chain will not be retargeted.", chain_name.c_str());
	      continue;
	    }
	  printChainVerbose(source_chain, chain_name + "_source");
	  
	  if( !target_tree.getChain(target_endpoints["begin"], target_endpoints["end"], target_chain) )
	    {
	      ROS_WARN("Failed to extract target chain for ( %s ). Chain will not be retargeted.", chain_name.c_str());
	      continue;
	    }
	  printChainVerbose(target_chain, chain_name + "_target");
	  
	  /// Get the order in which the joints occur in the chain, starting with the base joint. This is used to get data from joint state messages
	  _JointOrderMap source_joint_order_map;
	  int order = 0;
	  for(auto segment_it = source_chain.segments.begin(); segment_it != source_chain.segments.end(); ++segment_it)
	    {
	      _Joint const & joint = segment_it->getJoint();
	      
	      if( joint.getType() != _Joint::None )
		{
		  source_joint_order_map[joint.getName()] = order++;
		}
	    }

	  /// TIME TO LOAD THE JOINT LIMITS FOR THE TARGET! (We don't actually use these right now)
	  /// Number excludes fixed joints
	  unsigned int nr_target_joints = target_chain.getNrOfJoints();
	  _JntArray target_limits_upper(nr_target_joints), 
	    target_limits_lower(nr_target_joints);
	  
	  
	  ROS_INFO_STREAM( chain_name << " target joint limits: ");
	  
	  boost::shared_ptr<const urdf::Link> link = target_model.getLink(target_endpoints["end"]);
      	  boost::shared_ptr<const urdf::Joint> joint = target_model.getJoint(link->parent_joint->name);
	  
	  int i = 0;
      	  while(link->name != std::string(target_endpoints["begin"]))
      	    {
	      /**
	       * We don't apply joint angles to fixed joints when we retarget, so we exclude these.
	       * It's also worth noting that the KDL FK solvers do not accept joint angles for fixed joints.
	       */
      	      if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      		{
      		  double upper,lower;
		  
      		  joint = target_model.getJoint(link->parent_joint->name);
		  
      		  // get limits
      		  if (joint->type != urdf::Joint::CONTINUOUS)
      		    {
      		      lower = joint->limits->lower;
      		      upper = joint->limits->upper;
      		    }
      		  else
      		    {
      		      lower = 0.0;
      		      upper = 0.0;
      		    }
	    
      		  ROS_INFO_STREAM( '\t' << joint->name.c_str() << "( " << joint_type_text_map[joint->type] << " ): " << joint->limits->lower << " to " << joint->limits->upper );
	      
      		  // Add limits to joint array
		  target_limits_upper(i) = upper;
		  target_limits_lower(i) = lower;
		  ++i;
		}
      	      // jump one link lower in the chain
      	      link = target_model.getLink(link->getParent()->name);
      	    }
	  
	  /// Let's initialize the retargeter using our good friend aggregate initialization.
	  Retargeter retargeter
	  {
	    chain_name, target_to_source, source_chain, 
	    target_chain, source_joint_order_map,
	    target_limits_upper, target_limits_lower,
	    KDL::ChainFkSolverPos_recursive(source_chain),
	    rtk::PoseSolverLsq(target_chain)
	  };
	  
	  retargeters_.push_back(retargeter);
      }

  if ( !(retargeters_.size()) > 0 )
  {
    ROS_WARN("No retargeters could be loaded successfully.");
  }
 
  initPolicies<quickdev::policy::ALL>();
}
  
  
  QUICKDEV_SPIN_ONCE()
    {
      //
    }

  /// joint_state_msg is a boost::shared_ptr to a JointState message
  QUICKDEV_DECLARE_MESSAGE_CALLBACK2( sourceJointStateCB, _JointStateMsg, joint_state_msg )
    {
      /// TODO: Process the new joint state an retarget here
      
      std::vector<std::string> const & names = joint_state_msg->name;
      std::vector<double> const & positions = joint_state_msg->position;

      /// The message with the retargeted chain joint state that we will publish
      _JointStateMsg retargeted_joint_state;
      int nr_success = 0;

      /// Time to retarget
      for(auto retargeter_it = retargeters_.begin(); retargeter_it != retargeters_.end(); ++retargeter_it)
	{

	  std::string const & chain_name = retargeter_it->name_;
	  _JointOrderMap const & joint_order_map = retargeter_it->source_joint_order_map_;
	  
	  _Chain const & source_chain = retargeter_it->source_chain_;
	  _Chain const & target_chain = retargeter_it->target_chain_;
	  int nr_source_joints = source_chain.getNrOfJoints();
	  _JntArray source_state( nr_source_joints );

	  ROS_INFO("Retargeting [ %s ]", chain_name.c_str());
	  
	  /// Extract the joint angles corresponding to this chain
	  int nr_matches = 0;
	  auto name_it = names.begin();
	  for(auto position_it = positions.begin(); position_it != positions.end(); ++position_it, ++name_it)
	    {
	      /// Check if the current joint is part of this chain
	      auto match_it = joint_order_map.find( *name_it );
	      
	      if ( match_it != joint_order_map.end() )
		{
		  source_state( match_it->second ) = *position_it;
		  ++nr_matches;
		}
	    }
	  if( nr_matches != nr_source_joints )
	    {
	      ROS_WARN("Incoming joint state did not fully specify chain state.");
	      ROS_WARN("Joints in chain: %d, Matching joints received: %d", 
		       nr_source_joints, nr_matches);
	      ROS_WARN("Skipping chain...");
	      continue;
	    }
	  
	  _FrameArray source_frames;
	  if( rtk::spatial::forwardKinematics(source_chain, source_state, source_frames) )
	    {
	      ROS_WARN("Foward kinematics failed.");
	      ROS_WARN("Skipping chain...");
	      continue;
	    }
	  
	  rtk::spatial::transform(source_frames, retargeter_it->target_to_source_);
	  rtk::spatial::translateToOrigin(source_frames);
	  
	  /// TODO: Get these values (and maybe retargeting algorithm) from config
	  if( !retargeter_it->retargeter_.update(source_frames, 5, 0.001, 1000) )
	    {
	      ROS_WARN("KR solver failed. [ %s ]", chain_name.c_str() );
	      ROS_WARN("Skipping chain...");
	      continue;
	    }

	  _JntArray retargeted_angles = retargeter_it->retargeter_.getTargetAngles();

	  for(unsigned int i = 0; i < target_chain.getNrOfSegments(); ++i)
	    {
	      std::string joint_name = target_chain.getSegment(i).getJoint().getName();
	            
	      retargeted_joint_state.name.push_back(joint_name);
	      retargeted_joint_state.position.push_back(retargeted_angles(i));
	    }

	  ++nr_success;
	}

      if(nr_success > 0)
	{
	  multi_pub_.publish("retargeted_joint_states", retargeted_joint_state);
	  ROS_INFO( "Publishing joint states for %d chains.", nr_success );
	}
      
      return;
    }
  
 private:
  
  void printChainVerbose(_Chain const & chain, std::string const & name)
  {
    ROS_INFO_STREAM( "Loaded " << name << " :" );
    ROS_INFO_STREAM( "\tSegments: " << chain.getNrOfSegments() );
    ROS_INFO_STREAM( "\tJoints (excluding Joint::NONE): " << chain.getNrOfJoints() );
        
    for(auto segment_it = chain.segments.begin(); segment_it != chain.segments.end(); ++segment_it)
      {
	_Joint const & joint = segment_it->getJoint();
	_Frame const & f2t   = segment_it->getFrameToTip();
	ROS_INFO_STREAM( '\t' << joint.getName() << " (" <<  joint.getTypeName() << ") -> " 
			 << segment_it->getName() << " ( " << f2t.p.x() << ", " << f2t.p.y() 
			 << ", " << f2t.p.z() << " )" );
      }
  }
  
};

#endif // RSSDEMO_JOINTSTATERETARGETERNODE_H_
