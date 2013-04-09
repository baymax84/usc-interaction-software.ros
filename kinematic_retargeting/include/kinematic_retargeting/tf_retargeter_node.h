/***************************************************************************
 *  tf_retargeter_node.h
 *  --------------------
 *
 *  Copyright (c) 2013, Dylan J. Foster
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

#ifndef KINEMATICRETARGETING_TFRETARGETERNODE_H_
#define KINEMATICRETARGETING_TFRETARGETERNODE_H_

#include <quickdev/node.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>

/// data
#include <map>
#include <tuple>

/// tf
#include <tf/transform_listener.h>

/// robot model
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>

/// kinematics
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <rtk/math_impl.h>
#include <rtk/pose_solver_lsq.h>

/* This is version 1.9.44, Groovy. I'm not sure if this should actually have the Fuerte version */
/* The bottom line is that tf is changing around a bunch */
#if ROS_VERSION_MINIMUM(1, 9, 44)
  
  #include <kdl_conversions/kdl_msg.h>

  #define POSE_KDL_TO_MSG(__KDLFRAME, __POSEMSG) tf::poseKDLToMsg((__KDLFRAME), (__POSEMSG));
  #define TRANSFORM_TF_TO_KDL(__TFTRANSFORM, __KDLFRAME) tf::transformTFToKDL((__TFTRANSFORM), (__KDLFRAME))

#else

  #define POSE_KDL_TO_MSG(__KDLFRAME, __POSEMSG) tf::PoseKDLToMsg((__KDLFRAME), (__POSEMSG));
  #define TRANSFORM_TF_TO_KDL(__TFTRANSFORM, __KDLFRAME) tf::TransformTFToKDL((__TFTRANSFORM), (__KDLFRAME))

#endif

#include <tf_conversions/tf_kdl.h>

/// visualization
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>

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

/// visualization-related typedefs
typedef visualization_msgs::Marker      _MarkerMsg;
typedef visualization_msgs::MarkerArray _MarkerArrayMsg;
typedef std_msgs::ColorRGBA _ColorMsg;
typedef geometry_msgs::Pose _PoseMsg;

class TFCache
{
 public:
  
  // Map from the transform name to the transform and a bool representing whether or not the transform has successfully been loaded so far.
  typedef std::vector< std::tuple< std::string, bool, tf::Transform> > NamedTfStatArray;
  NamedTfStatArray name_frame_map_;
  
  tf::TransformListener tf_listener_;
  
 public:
  
  /// TODO: Clean up constructors
  TFCache() {}
  
  TFCache(XmlRpc::XmlRpcValue xml_names)
    {
      fromXmlRpc(xml_names);
    }

 TFCache(TFCache const & src):
  name_frame_map_( src.name_frame_map_)
    {
      
    }

  TFCache & operator=(const TFCache & rhs)
    {
      if(this == &rhs)
	return *this;
      else
	{
	  *this = rhs;
	  return *this;
	}
    }
  
  /** 
   * Expects a list of transform names.
   * 
   * @param xml_names
   * @param wait_time 
   * 
   * @return Zero if successful, non-zero otherwise
   */
  /// TODO: take const XmlRpcValue & instead of normal ref
  int fromXmlRpc(XmlRpc::XmlRpcValue & xml_names, int const & wait_time = 0.5)
  {
    for(int idx = 0; idx < xml_names.size(); ++idx)
      {
	std::string const tf_name = xml_names[ idx ];
	
	if (tf_listener_.canTransform( "/world", tf_name, ros::Time(0) ) )
	  {
	    tf::StampedTransform world_to_joint_tf;
	    
	    try
	      {
		tf_listener_.lookupTransform( "/world", tf_name, ros::Time(0), world_to_joint_tf);
	      }
	    catch(tf::TransformException ex)
	      {
		ROS_ERROR( "%s", ex.what() );

		/* name_frame_map_[ tf_name ] = std::make_pair(false, tf::Transform::getIdentity() ); */
		name_frame_map_.push_back( std::make_tuple( tf_name, false, tf::Transform::getIdentity() ) );
		continue;
	      }
	  
	    /* name_frame_map_[ tf_name ] = std::make_pair<bool, tf::Transform>(true, world_to_joint_tf); */
	    name_frame_map_.push_back( std::make_tuple( tf_name, true, tf::Transform(world_to_joint_tf) ) );
	    
	    ROS_INFO( "Loaded source frame [ %s ]", tf_name.c_str() );
	    
	  }
	else
	  {
	    ROS_WARN( "Initial lookup of [ %s ] failed.", tf_name.c_str() );
	    /* name_frame_map_[ tf_name ] = std::make_pair(false, tf::Transform::getIdentity() ); */
	    name_frame_map_.push_back( std::make_tuple( tf_name, false, tf::Transform::getIdentity() ) );
	    continue;
	  }
      }

    return 0;
  }

  int update()
  {
    int error = 0;

    /// Get newer versions of all of the transforms if they are available
    for( NamedTfStatArray::iterator tf_it = name_frame_map_.begin(); tf_it != name_frame_map_.end(); ++tf_it )
      {

	/* ROS_INFO("update reached frame: [ %s ]", std::get<0>( *tf_it).c_str()); */
	
	if( tf_listener_.canTransform( "/world", std::get<0>(*tf_it), ros::Time(0) ))
	  {
	    /* ROS_INFO("update can transform: [ %s ]", std::get<0>(*tf_it).c_str()); */
	    try
	      {
		tf::StampedTransform world_to_joint_tf;
	    
		tf_listener_.lookupTransform( "/world", std::get<0>(*tf_it), ros::Time(0), world_to_joint_tf);

		/* tf_it->second = std::make_pair( true, world_to_joint_tf ); */
		
		std::get<1>(*tf_it) = true;
		std::get<2>(*tf_it) = world_to_joint_tf;
		
	      }
	    catch(tf::TransformException ex)
	      {
		ROS_ERROR( "%s", ex.what() );
		error = -1;
	      }
	  }
	/* else */
	/*   ROS_WARN(" STILL CANT TRANSFORM %s", tf_it->first.c_str() ); */
	
      }
    
    return error;
  }

  bool allAvailable()
  {
    bool available = true;

    for( NamedTfStatArray::const_iterator tf_it = name_frame_map_.begin(); tf_it != name_frame_map_.end(); ++tf_it)
      {
	available &= std::get<1>( *tf_it );
      }
    
    return available;
  }

  // TODO: Fix this to use new data structure
  /* int getFrame(std::string const & frame_name, tf::Transform & out) const  */
  /* { */
  /*   NameTransformMap::const_iterator tf_it = name_frame_map_.find( frame_name ); */

  /*   if(  tf_it == name_frame_map_.end() ) */
  /*     return -1; */
  /*   else */
  /*     { */
  /* 	out = tf_it->second.second; */
  /* 	return (tf_it->second.first) ? 0 : -1; */
  /*     } */
  /* } */

  /// return all of the frames as an rtk::_FrameArray
  operator _FrameArray() const
  {
    _FrameArray output;
    
    for( NamedTfStatArray::const_iterator tf_it = name_frame_map_.begin(); tf_it != name_frame_map_.end(); ++tf_it )
      {
	_Frame world_to_joint_kdl;
	
	/* ROS_INFO("FrameArray output joint: [ %s ]", std::get<0>(*tf_it).c_str() ); */

	/* tf::transformTFToKDL( tf_it->second, world_to_joint_kdl ); */
	/* TRANSFORM_TF_TO_KDL( tf_it->second.second, world_to_joint_kdl ); */
	TRANSFORM_TF_TO_KDL( std::get<2>(*tf_it), world_to_joint_kdl );
	
	output.push_back( world_to_joint_kdl );
	
      }

    return output;
  }

};

struct Retargeter
{
  std::string name_;
  _Frame target_to_source_;
  
  /* _Chain source_chain_; */
  TFCache source_frames_;
  _Chain target_chain_;

  _JntArray target_limits_upper_;
  _JntArray target_limits_lower_;

  rtk::PoseSolverLsq retargeter_;

};

QUICKDEV_DECLARE_NODE( TFRetargeter )

QUICKDEV_DECLARE_NODE_CLASS( TFRetargeter )
{
 protected:
  /// communications
  ros::MultiPublisher<> multi_pub_;
  ros::MultiSubscriber<> multi_sub_;
  
  /// retargeting functionality
  /// Using deque instead of vector because std::vector requires that its storage type have a default constructor and an assignment operator
  std::deque<Retargeter> retargeters_;
  
  /// For visualization
  _ColorMsg
    red_template_,
    blue_template_;

  /// Used in retargeting algorithm 
  double ee_weight_;
  /* double cost_weight_; */

  tf::StampedTransform world_to_head_tf_;
  
  QUICKDEV_DECLARE_NODE_CONSTRUCTOR( TFRetargeter )
  {
    world_to_head_tf_.setData(tf::Transform::getIdentity());
  }
  
  
  QUICKDEV_SPIN_FIRST()
    {
      QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );
      
      red_template_.a = 1.0;
      red_template_.r = 1.0;
      red_template_.b = 0.0;
      red_template_.g = 0.0;
      blue_template_.a = 1.0;
      blue_template_.r = 0.0;
      blue_template_.b = 1.0;
      blue_template_.g = 0.0;
      
      /// Initialize communications
      multi_pub_.addPublishers<_JointStateMsg, _MarkerArrayMsg>( nh_rel, { "retargeted_joint_states", "visualization_marker_array" } );

      /// Get end effector weight
      nh_rel.param<double>("ee_weight", ee_weight_, 50.0);
      /* nh_rel.param<double>("cost_weight", cost_weight_, 0.001); */
      
      /// Get URDF path
      std::string target_urdf_path;
      
      if(!nh_rel.getParam("target_urdf", target_urdf_path))
	{
	  ROS_ERROR("Couldn't retrieve target urdf path from parameter server.");
	  exit(1);
	}
      
      /// Load the trees
      urdf::Model target_model;

      if (!target_model.initFile(target_urdf_path))
	{
	  ROS_ERROR("Failed to initialize target URDF ( %s ).", target_urdf_path.c_str());
	  exit(1);
	}
      else
	ROS_INFO("Initialized URDF file succesfully." );

      ROS_INFO("Loading kinematic trees...");
      _Tree target_tree;
      
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
	  _Chain target_chain;

	  TFCache source_frames;

	  /// The name of the chain being retargeted
	  std::string chain_name = chain_it->first;
	  XmlRpc::XmlRpcValue & source_frame_list = chain_it->second["source"];
	  XmlRpc::XmlRpcValue & target_endpoints = chain_it->second["target"];
	  XmlRpc::XmlRpcValue & t2s_xml = chain_it->second["target_to_source"];
	  
	  /// TODO: This, properly
	  /// Init the source frame tf cache
	  /// Should print tf frames as they are loaded
	  if ( source_frames.fromXmlRpc( source_frame_list ) )
	    ROS_WARN( "Failed to initialize source frame cache.");

	  _Frame target_to_source = _Frame(_Rotation::RPY(t2s_xml["roll"], t2s_xml["pitch"], t2s_xml["yaw"]),
					   _Vector(t2s_xml["x"], t2s_xml["y"], t2s_xml["z"]));
	  
	  /* ROS_INFO_STREAM( chain_name << " endpoints: " ); */
	  /* ROS_INFO_STREAM( "source:"); */
	  /* ROS_INFO_STREAM( "\t" << source_endpoints["begin"] */
	  /* 		   << " ----> " */
	  /* 		   << source_endpoints["end"]); */
	  ROS_INFO_STREAM( "target endpoints:" );
	  ROS_INFO_STREAM( "\t" << target_endpoints["begin"]
			   << " ----> "
			   << target_endpoints["end"] );
	  ROS_INFO("--------------------");
	  
	  if( !target_tree.getChain(target_endpoints["begin"], target_endpoints["end"], target_chain) )
	    {
	      ROS_WARN("Failed to extract target chain for ( %s ). Chain will not be retargeted.", chain_name.c_str());
	      continue;
	    }
	  printChainVerbose(target_chain, chain_name + "_target");
	  
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
	      // Traverse the chain backwards starting at the end effector
	      joint = target_model.getJoint(link->parent_joint->name);

	      double upper = 0,lower = 0;
	      
	      /**
	       * We don't apply joint angles to fixed joints when we retarget, so we exclude these.
	       * It's also worth noting that the KDL FK solvers do not accept joint angles for fixed joints.
	       */
      	      if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      		{
     		  	  
      		  // get limits
      		  if (joint->type != urdf::Joint::CONTINUOUS )
      		    {
      		      lower = joint->limits->lower;
      		      upper = joint->limits->upper;
      		    }
      		  else
      		    {
      		      lower = 0.0;
      		      upper = 0.0;
      		    }
	  
      		  // Add limits to joint array
		  target_limits_upper(i) = upper;
		  target_limits_lower(i) = lower;
		  ++i;
		}

	      ROS_INFO_STREAM( '\t' << joint->name.c_str() << "( " << joint_type_text_map[joint->type] << " ): " << lower << " to " << upper );

      	      // jump one link lower in the chain
      	      link = target_model.getLink(link->getParent()->name);
      	    }
	  /// TODO: fix tfcache part
	  Retargeter retargeter
	  {
	    chain_name, target_to_source,
	    source_frames, target_chain,
	    target_limits_upper, target_limits_lower,
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
      QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

      /// TODO: Don't check params this often / use dynamic reconfigure
      nh_rel.param<double>("ee_weight", ee_weight_, 50.0);
      /* nh_rel.param<double>("cost_weight", cost_weight_, 0.001); */

      ros::Time now = ros::Time::now();
      
      /// The message with the retargeted chain joint state that we will publish
      _JointStateMsg retargeted_joint_state;
      int nr_success = 0;

      /// Time to retarget
      for(auto retargeter_it = retargeters_.begin(); retargeter_it != retargeters_.end(); ++retargeter_it)
      	{
      	  std::string const & chain_name = retargeter_it->name_;

	  _Chain const & target_chain = retargeter_it->target_chain_;
	  
	  /// Update the source tf frames ------------------------------------
	  
	  retargeter_it->source_frames_.update();

	  if ( !retargeter_it->source_frames_.allAvailable() )
	    {
	      ROS_WARN( "TF frames for source chain [ %s ] are unavailable.", chain_name.c_str() );
	      continue;
	    }

	  _FrameArray source_frames = retargeter_it->source_frames_;

	  ROS_INFO("Retargeting [ %s ]", chain_name.c_str());
	  
	  /// Retarget ------------------------------------
      	  	  
      	  rtk::spatial::transform(source_frames, retargeter_it->target_to_source_);
      	  rtk::spatial::translateToOrigin(source_frames);
	  /* rtk::spatial::translate(source_frames, retargeter_it->target_to_source_.p); */
	  
      	  /// TODO: Get these values (and maybe retargeting algorithm) from config
      	  if( !retargeter_it->retargeter_.update(source_frames, ee_weight_, 0.00001, 10000) )
      	    {
      	      ROS_WARN("KR solver failed. [ %s ]", chain_name.c_str() );
      	      ROS_WARN("Skipping chain...");
      	      continue;
      	    }

      	  _JntArray retargeted_angles = retargeter_it->retargeter_.getTargetAngles();

	  /// Run the cost function again and publish the final results
      	  _FrameArray retargeted_frames;
      	  rtk::spatial::forwardKinematics(target_chain,retargeted_angles, retargeted_frames);
	  
	  float mse = rtk::cost::LSQUniform(source_frames, retargeted_frames);
	    
	  ROS_INFO("Chain MSE: %f", mse);
	  
	  /* if(mse > 180) */
	  /*   { */
	  /*     ROS_WARN("MSE is too high. Ignoring solution..."); */
	  /*     continue; */
	  /*   } */

      	  /// A bad way of ignoring the joint state of the end effector
      	  /// The algorith currently optimizes over this angle, which it shoulnd't do as it doesn't affect the position of any
      	  /// joints in the chain
      	  for(unsigned int i = 0; i < target_chain.getNrOfSegments()-1; ++i)
      	    {
      	      std::string joint_name = target_chain.getSegment(i).getJoint().getName();
	            
      	      retargeted_joint_state.name.push_back(joint_name);
      	      retargeted_joint_state.position.push_back(retargeted_angles(i));

      	    }

      	    
	  
      	  ++nr_success;
	  
#if KINEMATICRETARGETING_TFRETARGETERNODE_DEBUG_ALL

     	  ROS_INFO("visualizing source chain...");
      	  visualizeFrameArray(source_frames, chain_name + "_source", blue_template_);
      	  visualizeFrameArray(rtk::spatial::normalize(source_frames),
      			      chain_name + "_source_normalized", blue_template_);
      	  _JntArray target_zero(target_chain.getNrOfJoints());

      	  ROS_INFO("visualizing untargeted chain..s.");
      	  visualizeKDLChain(target_chain, target_zero,chain_name + "_untargeted", red_template_);
      	  ROS_INFO("visualizing retargeted chain...");

      	  visualizeFrameArray(retargeted_frames, chain_name + "_retargeted", red_template_);
      	  visualizeFrameArray(rtk::spatial::normalize(retargeted_frames),
      			      chain_name + "_retargeted_normalized", red_template_);
      	  /// visualize all of the chains involved in this algorithm

#elif KINEMATICRETARGETING_TFRETARGETERNODE_DEBUG_MIN
 	  visualizeFrameArray(rtk::spatial::normalize(source_frames),
      			      chain_name + "_source_normalized", blue_template_);
	  visualizeFrameArray(rtk::spatial::normalize(retargeted_frames),
      			      chain_name + "_retargeted_normalized", red_template_);
#endif
	  
      	}

      /* Bad head tracking */
      tf::TransformListener tf_listener;
      
      	if (tf_listener.canTransform( "/world", "head", ros::Time(0) ) )
	  {
	    
	    try
	      {
		tf_listener.lookupTransform( "/world", "head", ros::Time(0), world_to_head_tf_);
	      }
	    catch(tf::TransformException ex)
	      {
		ROS_ERROR( "%s", ex.what() );
	      }
	  
	  }

	double yaw, pitch, roll;
	world_to_head_tf_.getBasis().getEulerZYX(yaw, pitch, roll);

	retargeted_joint_state.name.push_back("HeadTurn");
	retargeted_joint_state.position.push_back(yaw);

	/* retargeted_joint_state.name.push_back("HeadNod"); */
	/* retargeted_joint_state.position.push_back(-pitch); */

	/* End bad head tracking */

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

  int visualizeKDLChain(const KDL::Chain & chain , const KDL::JntArray & angles, const std::string & marker_namespace, _ColorMsg const & color)
  {
    _FrameArray chain_pose;
  
    rtk::spatial::forwardKinematics(chain, angles, chain_pose);
  
    visualizeFrameArray(chain_pose, marker_namespace, color);
  
    return 0;
  }

  int visualizeFrameArray(_FrameArray pose, const std::string & marker_namespace, _ColorMsg const & color)
  {
    QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );
  
    _MarkerMsg 
      marker_template,
      joints_marker_template,
      text_marker_template,
      line_marker_template;
    
    std::string world_frame_name;
    nh_rel.param<std::string>("world_frame_name", world_frame_name, "/world");
  
    marker_template.header.frame_id = world_frame_name;
  
    marker_template.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_template.action = visualization_msgs::Marker::ADD;
    marker_template.lifetime = ros::Duration(1.0);
    marker_template.pose.orientation.w = 1.0;
    marker_template.color = color;
    marker_template.scale.x = 
      marker_template.scale.y = 
      marker_template.scale.z = .05;
  
    joints_marker_template = marker_template;
    joints_marker_template.type = visualization_msgs::Marker::SPHERE_LIST;
    /* joints_marker_template.scale.x =  */
    /* 	joints_marker_template.scale.y =  */
    /* 	joints_marker_template.scale.z = .05; */
  
    text_marker_template = marker_template;
    text_marker_template.type= visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker_template.scale.z = 0.1;
  
    line_marker_template = marker_template;
    line_marker_template.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker_template.scale.x = 0.02;
  
    auto now = ros::Time::now();
  
    _PoseMsg marker_pose;
  
    KDL::Frame pose_centroid = rtk::spatial::getCentroid(pose);
  
    _MarkerArrayMsg markers;
    _MarkerMsg joint_marker(joints_marker_template);
    _MarkerMsg text_marker(text_marker_template);
    _MarkerMsg chain_name_marker(text_marker_template);
    _MarkerMsg segment_marker(line_marker_template);
    _MarkerMsg ee_marker(marker_template);
    unsigned int id = 0;
  
    /* marker = joints_marker_template; */
    joint_marker.header.stamp = now;
    joint_marker.ns = marker_namespace;
    joint_marker.id = id++;
    joint_marker.color = color;
  
    text_marker.header.stamp = now;
    text_marker.ns = marker_namespace;
    text_marker.color = color;
  
    chain_name_marker.header.stamp = now;
    chain_name_marker.ns = marker_namespace;
    chain_name_marker.id = id++;
    chain_name_marker.color = color;
  
    segment_marker.header.stamp = now;
    segment_marker.ns = marker_namespace;
    segment_marker.id = id++;
    segment_marker.color = color;
  
    // TODO: Chamnge end effector marker to an arrow pointing along the segment to the end effector
    
    ee_marker.header.stamp = now;
    ee_marker.type = visualization_msgs::Marker::CUBE;
    ee_marker.ns = marker_namespace;
    ee_marker.id = id++;
    ee_marker.color = color;

    for(auto frame_it = pose.begin(); frame_it != pose.end(); ++frame_it, ++id)
      {

	/* tf::poseKDLToMsg(*frame_it, marker_pose); */
	POSE_KDL_TO_MSG( *frame_it, marker_pose );
	
	joint_marker.points.push_back(marker_pose.position);
	segment_marker.points.push_back(marker_pose.position);
	
      }
    markers.markers.push_back(joint_marker);
    markers.markers.push_back(segment_marker);

   /* tf::poseKDLToMsg(pose.back(), marker_pose); */
    POSE_KDL_TO_MSG( pose.back(), marker_pose );

    
    ee_marker.pose = marker_pose;
    
    markers.markers.push_back(ee_marker);

    chain_name_marker.text = marker_namespace;
    
    /* tf::poseKDLToMsg(pose_centroid, marker_pose); */
    POSE_KDL_TO_MSG( pose_centroid, marker_pose );
    
    chain_name_marker.pose = marker_pose;
    chain_name_marker.pose.position.z += 0.4;
    
    markers.markers.push_back(chain_name_marker);
    
    multi_pub_.publish("visualization_marker_array", markers );
        
    return 0;
  }

  
};

#endif // KINEMATICRETARGETING_TFRETARGETERNODE_H_
