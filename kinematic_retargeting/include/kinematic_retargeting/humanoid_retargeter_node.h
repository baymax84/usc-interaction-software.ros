/***************************************************************************
 *  include/kinematic_retargeting/humanoid_retargeter_node.h
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

#ifndef KINEMATICRETARGETING_HUMANOIDRETARGETERNODE_H_
#define KINEMATICRETARGETING_HUMANOIDRETARGETERNODE_H_

// STL
#include <map>

// Robot model data parsing
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

// humanoid 
#include <humanoid_recognizers/humanoid_recognizer_policy.h>

// kdl parser
#include <kdl_parser/kdl_parser.hpp>

// quickdev
#include <quickdev/node.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>
#include <quickdev/tf_tranceiver_policy.h>
#include <quickdev/reconfigure_policy.h>

// kinematic retargeting toolkit
#include <krt/math_impl.h>
#include <krt/pose_solver_lsq.h>

// minimization library
#include <gsl/gsl_multimin.h>

// messages
#include <std_msgs/ColorRGBA.h>

#include <kinematic_retargeting/humanoid_conversions.h>

// dynamic reconfigure
#include <kinematic_retargeting/HumanoidRetargeterConfig.h>

typedef std::map<_JointName, _JointName>  _EndpointMap;
typedef std::vector<_Chain>               _ChainArray;

typedef sensor_msgs::JointState                   _JointStateMsg;
typedef HumanoidRecognizerPolicy<_JointStateMsg>  _HumanoidRecognizerPolicy;

typedef visualization_msgs::Marker      _MarkerMsg;
typedef visualization_msgs::MarkerArray _MarkerArrayMsg;
typedef std_msgs::ColorRGBA             _ColorMsg;
typedef geometry_msgs::Pose             _PoseMsg;

typedef kinematic_retargeting::HumanoidRetargeterConfig       _HumanoidRetargeterConfig;
typedef quickdev::ReconfigurePolicy<_HumanoidRetargeterConfig> _HumanoidRetargeterLiveParams;

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

typedef struct
{
  std::string name_;
  _Frame target_to_source_;
  _EndpointMap endpoints_;
  
} _RetargetingMapping;

typedef std::vector<_RetargetingMapping> _RetargetingMappingList;


// TODO: Use xmlrpc and rosparam to avoid using yaml_cpp api directly (because it sucks)

static void operator >> (const YAML::Node& node, _RetargetingMapping & chain)
{
  node["name"] >> chain.name_;
  
  const YAML::Node & source = node["source"];
  const YAML::Node & target = node["target"];
  const YAML::Node &    t2s = node["target_to_source"];
  
  source["parent"] >> chain.endpoints_["source_parent"];
  source["begin"] >> chain.endpoints_["source_begin"];
  source["end"]   >> chain.endpoints_["source_end"];

  target["begin"] >> chain.endpoints_["target_begin"];
  target["end"]   >> chain.endpoints_["target_end"];
  
  double x,y,z,roll,pitch,yaw;
  
  t2s["x"] >> x;
  t2s["y"] >> y;
  t2s["z"] >> z;
  t2s["roll"] >> roll;
  t2s["pitch"] >> pitch;
  t2s["yaw"] >> yaw;

  chain.target_to_source_ = _Frame(_Rotation::RPY(roll,pitch,yaw), _Vector(x,y,z));

  return;
}
    
static int loadYamlChainDef(std::string const &filename, _RetargetingMappingList & chains)
{
  std::ifstream fin;
  fin.open(filename.c_str());
  if (fin.fail())
    {
      ROS_WARN("Failed to load chain definition file: %s", filename.c_str());
      return 1;
    }
    
  YAML::Node doc;
  YAML::Parser parser(fin);

  try{
    parser.GetNextDocument(doc);
  }

  catch(YAML::ParserException& e)
    {
      ROS_WARN("Failed to parse chain definition file");
      std::cout << e.what() << std::endl;
      fin.close();
      return 1;
    }

  std::cout << "Chain def has size " << doc.size() << std::endl;
  for(unsigned int i = 0; i < doc.size(); ++i)
    {
      _RetargetingMapping chain;
      doc[i] >> chain;
      chains.push_back(chain);
    }

  fin.close();
  return 0;
}

QUICKDEV_DECLARE_NODE( HumanoidRetargeter, _HumanoidRecognizerPolicy, _TfTranceiverPolicy, _HumanoidRetargeterLiveParams)
// Declare a class called HumanoidRetargeterNode
//
QUICKDEV_DECLARE_NODE_CLASS( HumanoidRetargeter )
{
 protected:
  quickdev::MutexedCache<std::deque<_HumanoidStateArrayMsg::ConstPtr> > state_arrays_cache_;

  // Communications
  ros::MultiPublisher<> multi_pub_;
  ros::MultiSubscriber<> multi_sub_;

  // Robot model
  urdf::Model model_;

  _ChainArray target_chains_;

  // target joint limits
  _JntArray target_min_;
  _JntArray target_max_;
  
  // Target chain endpoints
  _RetargetingMappingList retargetings_;

  std::string world_frame_name_ ;
  
  _MarkerMsg 
    marker_template_,
    joints_marker_template_,
    text_marker_template_,
    line_marker_template_;
  
  _ColorMsg
    red_template_,
    blue_template_;

  QUICKDEV_DECLARE_NODE_CONSTRUCTOR( HumanoidRetargeter )
  {
  }
  
  QUICKDEV_SPIN_FIRST()
    {
      // Get nodehandle
      QUICKDEV_GET_RUNABLE_NODEHANDLE(nh_rel);
      
      _HumanoidRecognizerPolicy::registerCallback( quickdev::auto_bind( &HumanoidRetargeterNode::humanoidStatesCB, this ) );
      
      // Set up communications
      multi_pub_.addPublishers<_JointStateMsg>(nh_rel, {"joint_states"});


      
      /* Load yaml chain definitions -------------------- */
      
      std::string chain_def_uri = ros::ParamReader<std::string, 1>::readParam(nh_rel, "chain_defs", "");
      std::cout << "Loading chain endpoint definitions in " <<  chain_def_uri << "..." << std::endl;
      
      if(loadYamlChainDef(chain_def_uri, retargetings_))
	{
	  ROS_ERROR("Failed to load chain endpoint definitions");
	  exit(1);
	}

      std::cout << "Loaded " << retargetings_.size() 
		<< " endpoint definitions" << std::endl;
      for(auto chain_it = retargetings_.begin(); chain_it != retargetings_.end(); ++chain_it)
	{
	  std::cout << chain_it->name_ << ':' << std::endl;
	  std::cout << "source:" << std::endl;
	  std::cout << '\t' << chain_it->endpoints_["source_begin"]
	  	    << " ----> "
	  	    << chain_it->endpoints_["source_end"]
	  	    << std::endl;
	  std::cout << "target:" << std::endl;
	  std::cout << '\t' << chain_it->endpoints_["target_begin"]
	  	    << " ----> "
	  	    << chain_it->endpoints_["target_end"]
	  	    << std::endl;
	}
      std::cout << "--------------------" << std::endl;
       
      /* Load robot description -------------------- */

      const std::string urdf_uri = ros::ParamReader<std::string, 1>::readParam( nh_rel, "urdf_uri", "" );
      std::cout << "URDF: " << urdf_uri << std::endl; 

      if(!model_.initFile(urdf_uri))
        {
          ROS_ERROR("Failed to Parse URDF");
        } 
      
      std::cout << "Loading URDF File" << std::endl;
      
      /* Print model joints -------------------- */

      std::cout << "Read " << model_.joints_.size() << " joints" << std::endl;
      for( auto joint_it = model_.joints_.cbegin(); joint_it != model_.joints_.cend(); ++joint_it )
        {     
	  std::cout << joint_it->first << std::endl;
        }
      std::cout << "--------------------" << std::endl;
      
      /* Generate kinematic tree from URDF -------------------- */

      KDL::Tree kinematic_tree;
      
      std::cout << "Generating kinematic tree" << std::endl;
      if (!kdl_parser::treeFromUrdfModel(model_, kinematic_tree))
	ROS_ERROR("Failed to generate tree");
      
      auto const num_segments = kinematic_tree.getNrOfSegments();

      // Print tree segments
      std::cout << "Tree has " << num_segments << " segments." << std::endl;
      auto const & segment_map = kinematic_tree.getSegments();
      for(auto segment_it = segment_map.cbegin(); segment_it != segment_map.cend(); ++segment_it)
	{
	  std::cout << segment_it->first << std::endl;
	} 
      std::cout << "--------------------" << std::endl;      
      

      // Generate chains to be retargeted
      std::cout << "Generating chains..." << std::endl << std::endl;
      for(auto chain_it = retargetings_.begin(); chain_it != retargetings_.end(); ++chain_it)
	{
	  KDL::Chain target_chain;
	  _JointName target_begin, target_end;

	  target_begin = chain_it->endpoints_["target_begin"];
	  target_end   = chain_it->endpoints_["target_end"];
	  
	  // Get target chain
	  if(!kinematic_tree.getChain(target_begin, target_end, target_chain))
	    {
	      ROS_ERROR("Failed to initialize target chain.");
	    }
	  
	  for(unsigned int i =0; i < target_chain.getNrOfSegments(); ++i)
	    {
	      _Segment segment = target_chain.getSegment(i);
	      _Frame f2t = segment.getFrameToTip();
	      _JointName name = segment.getName();

	      ROS_INFO("Iteration: %d, Segment: %s, Frame to tip: (%f, %f, %f)", i, name.c_str(), f2t.p.x(), f2t.p.y(), f2t.p.z());
	      
	    }

	  target_chains_.push_back(target_chain);
	  
	  // Print target chain
	  std::cout << "Loaded target chain: " << chain_it->name_ << std::endl;
	  std::cout << "\tSegments: " << target_chain.getNrOfSegments() << std::endl;
	  std::cout << "\tJoints: " << target_chain.getNrOfJoints() << std::endl;
	  
	  std::cout << chain_it->name_ << ':' << std::endl;
	  for(auto segment_it = target_chain.segments.begin(); segment_it != target_chain.segments.end(); ++segment_it)
	    {
	      auto const & joint = segment_it->getJoint();
	      std::cout << '\t' << segment_it->getName() << " -> " << joint.getName() << std::endl;
	    }

	  // Get target chain limits from urdf
	  std::cout << "limits:" << std::endl;
      
	  boost::shared_ptr<const urdf::Link> link = model_.getLink(target_end); 
	  boost::shared_ptr<const urdf::Joint> joint = model_.getJoint(link->parent_joint->name);
	  while(link->name != target_begin)
	    {
	      if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
		{
		  double upper,lower;
	  
		  joint = model_.getJoint(link->parent_joint->name);
	  
		  // get limits
		  if (joint->type != urdf::Joint::CONTINUOUS)
		    {
		      lower = joint->limits->lower;
		      upper = joint->limits->upper;
		    }
		  else
		    {
		      lower = 0;
		      upper = 0;
		    }
	    
		  std::cout << '\t' << joint->name.c_str() << ": " << joint->limits->lower << " to " << joint->limits->upper << std::endl;
	      
		  // Add limits to joint array
		  target_min_.resize(target_min_.rows()+1);
		  target_max_.resize(target_max_.rows()+1);
		  target_min_.data[target_min_.rows()-1] = lower;
		  target_max_.data[target_max_.rows()-1] = upper;
		}
	      // jump one link lower in the chain
	      link = model_.getLink(link->getParent()->name);
	    }
	  std::cout << std::endl;
	}
      std::cout << "--------------------" << std::endl;

      // initialize quickdev policies
      initPolicies<_HumanoidRecognizerPolicy>( "update_humanoids_param", false );
      initPolicies<quickdev::policy::ALL>();

      /* Initialize marker templates -------------------- */
      world_frame_name_ = quickdev::ParamReader::readParam<decltype( world_frame_name_ )>( nh_rel, "world_frame_name", "/world" );
											
      double loop_rate_hz = quickdev::ParamReader::readParam<decltype(loop_rate_hz)>(nh_rel, "loop_rate", 10);
      
      red_template_.a = 1.0;
      red_template_.r = 1.0;
      red_template_.b = 0.0;
      red_template_.g = 0.0;
      blue_template_.a = 1.0;
      blue_template_.r = 0.0;
      blue_template_.b = 1.0;
      blue_template_.g = 0.0;

      marker_template_.header.frame_id = world_frame_name_;
      
      marker_template_.type = visualization_msgs::Marker::SPHERE_LIST;
      marker_template_.action = visualization_msgs::Marker::ADD;
      marker_template_.lifetime = ros::Duration(1.0);
      marker_template_.pose.orientation.w = 1.0;
      marker_template_.color = red_template_;
      marker_template_.scale.x = 
	marker_template_.scale.y = 
	marker_template_.scale.z = .05;
      
      joints_marker_template_ = marker_template_;
      joints_marker_template_.type = visualization_msgs::Marker::SPHERE_LIST;
      /* joints_marker_template_.scale.x =  */
      /* 	joints_marker_template_.scale.y =  */
      /* 	joints_marker_template_.scale.z = .05; */
      
      text_marker_template_ = marker_template_;
      text_marker_template_.type= visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker_template_.scale.z = 0.1;

      line_marker_template_ = marker_template_;
      line_marker_template_.type = visualization_msgs::Marker::LINE_STRIP;
      line_marker_template_.scale.x = 0.02;
      

    }

  int visualizeKDLChain(const _Chain & chain, const _JntArray & angles, const std::string & marker_namespace, _ColorMsg const & color)
  {
    _HumanoidStateMsg joint_position;
    
    forwardKinematics(chain, angles, joint_position);
    
    visualizeHumanoidState(joint_position, marker_namespace, color);
    
    return 0;
  }
  
  // TODO: Mofiy this function such that orientation of joints is also visualized
  int visualizeFrameArray(_FrameArray pose, const std::string & marker_namespace, _ColorMsg const & color)
  {
    auto now = ros::Time::now();

    _PoseMsg marker_pose;

    _Frame pose_centroid = krt::spatial::getCentroid(pose);
    
    _MarkerArrayMsg markers;
    _MarkerMsg joint_marker(joints_marker_template_);
    _MarkerMsg text_marker(text_marker_template_);
    _MarkerMsg chain_name_marker(text_marker_template_);
    _MarkerMsg segment_marker(line_marker_template_);
    _MarkerMsg ee_marker(marker_template_);
    unsigned int id = 0;
    
    /* marker = joints_marker_template_; */
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
	tf::PoseKDLToMsg(*frame_it, marker_pose);
	
	joint_marker.points.push_back(marker_pose.position);
	segment_marker.points.push_back(marker_pose.position);
	
     }
    markers.markers.push_back(joint_marker);
    markers.markers.push_back(segment_marker);

    tf::PoseKDLToMsg(pose.back(), marker_pose);
    
    ee_marker.pose = marker_pose;
    
    markers.markers.push_back(ee_marker);

    chain_name_marker.text = marker_namespace;
    
    tf::PoseKDLToMsg(pose_centroid, marker_pose);
    chain_name_marker.pose = marker_pose;
    chain_name_marker.pose.position.z += 0.4;
    
    markers.markers.push_back(chain_name_marker);

    _HumanoidRecognizerPolicy::updateMarkers(markers);
        
    return 0;
  }
  
    // TODO: Mofiy this function such that orientation of joints is also visualized
    int visualizeHumanoidState(_HumanoidStateMsg const & state, std::string const & marker_namespace, _ColorMsg const & color)
  {
    auto now = ros::Time::now();
    
    _MarkerArrayMsg markers;
    _MarkerMsg joint_marker(joints_marker_template_);
    _MarkerMsg text_marker(text_marker_template_);
    _MarkerMsg chain_name_marker(text_marker_template_);
    _MarkerMsg segment_marker(line_marker_template_);
    _MarkerMsg ee_marker(marker_template_);
    unsigned int id = 0;
    
    /* marker = joints_marker_template_; */
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
    
    /* ROS_INFO("Entering for loop..."); */
    for(auto joint_it = state.joints.begin(); joint_it != state.joints.end(); ++joint_it, ++id)
      {
	/* ROS_INFO(" inside viz loop id=%d", id); */
	joint_marker.points.push_back(joint_it->pose.pose.position);
	segment_marker.points.push_back(joint_it->pose.pose.position);
	
	/* if(config_.show_joint_names) */
	/*   { */
	/*     text_marker.text = joint_it->name; */
	/*     text_marker.id = id; */
	/*     text_marker.pose.position = joint_it->pose.pose.position; */
	/*     text_marker.pose.position.z += 0.2; */
	/*     markers.markers.push_back(text_marker); */
	/*   } */
      }
    markers.markers.push_back(joint_marker);
    markers.markers.push_back(segment_marker);

    ee_marker.pose.position = state.joints.back().pose.pose.position;
    
    markers.markers.push_back(ee_marker);

    // Place the chain's name over its joint centroid
    chain_name_marker.text = marker_namespace;
    chain_name_marker.pose.position = state.joints.back().pose.pose.position;
    chain_name_marker.pose.position.z += 0.4;
    
    markers.markers.push_back(chain_name_marker);

    _HumanoidRecognizerPolicy::updateMarkers(markers);
    
    return 0;
  }
  
  QUICKDEV_SPIN_ONCE()
    {
      // get a read/write reference to the cache's data (called state_arrays_cache)
      QUICKDEV_LOCK_CACHE_AND_GET( state_arrays_cache_, state_arrays_cache );

      auto const now = ros::Time::now();
      
      
      // given the desired "world" frame, transform all the joints of all incoming humanoids to this world frame, then mesh them together into one message
      for( auto users_msg_it = state_arrays_cache.begin(); users_msg_it != state_arrays_cache.end(); ++users_msg_it )
        {
	  // copy
	  _HumanoidStateArrayMsg users_msg = *(*users_msg_it);
	  auto & users = users_msg.states;

	  for( auto user_it = users.begin(); user_it != users.end(); ++user_it )
            {
	      auto & joints = user_it->joints;

	      for( auto joint_it = joints.begin(); joint_it != joints.end(); ++joint_it )
                {
		  auto & joint = *joint_it;

		  // if the joint was already measured with respect to the world frame, then move to the next joint
		  if( joint.header.frame_id == world_frame_name_ ) continue;

		  // otherwise, transform the joint into the coordinate frame specified

		  // the joint was measured with respect to some sensor; get that transform
		  auto const sensor_to_joint_tf = unit::convert<btTransform>( joint.pose.pose );
		  // the sensor should exist somewhere in the world; look up that transform
		  auto const world_to_sensor_tf = _TfTranceiverPolicy::waitForAndLookupTransform( world_frame_name_, joint.header.frame_id, 10, 10 );

		  joint.pose.pose = unit::make_unit( world_to_sensor_tf * sensor_to_joint_tf );
		  joint.header.frame_id = world_frame_name_;
                }
            }

	  // mesh all parts of all humanoids into one message
	  _HumanoidRecognizerPolicy::updateHumanoids( users_msg );
        }

      /* Erase all messages older than 2.0s  -------------------- */
      _HumanoidRecognizerPolicy::eraseOld(2.0);
      
      /* Get humanoid message -------------------- */
	
      auto & humanoids = _HumanoidRecognizerPolicy::getHumanoids();
      
      ROS_INFO("Tracking %d humanoids", humanoids.size());
      if(humanoids.size() == 0)
	return;
	
      auto humanoid_it = humanoids.begin();
      auto & humanoid = *humanoid_it;
      
#ifdef KR_DEBUG_HUMANOID
      auto joints = humanoid.getJointsMessage();
      
      _MarkerArrayMsg markers;
      _MarkerMsg marker;
	
      marker = joints_marker_template_;
       
      marker.ns = "kr_test_all";
      marker.id = 0;
       
      for( auto joint = joints.joints.cbegin(); joint != joints.joints.cend(); ++joint )
	{
	  marker.points.push_back( joint->pose.pose.position );
	}
       
      markers.markers.push_back(marker);
      _HumanoidRecognizerPolicy::updateMarkers(markers);
#endif

#ifdef KR_DEBUG_KDL_VIZ
      std::cout << "About to visualize dummy chain..." << std::endl;
      _Chain source;
      _Joint sj0 = _Joint(_Joint::RotZ);
      _Frame sf0 = _Frame(_Vector(0.361, 0.0, 0.0));
      _Segment ss0 = _Segment(sj0, sf0);
      source.addSegment(ss0);
      _Joint sj1 = sj0;
      _Frame sf1 = _Frame(_Vector(0.541, 0.0, 0.0));
      _Segment ss1 = _Segment(sj1, sf1);
      source.addSegment(ss1);
      _Joint sj2 = sj1;
      _Frame sf2 = _Frame(_Vector(0.541, 0.0, 0.0));
      _Segment ss2 = _Segment(sj2, sf2);
      source.addSegment(ss2);
      _JntArray source_angles = _JntArray(3);
      source_angles(0) = 0.0;
      source_angles(1) = 0.0;
      source_angles(2) = 0.0;
      visualizeKDLChain(source, source_angles, "fixed_chain", red_template_);
#endif
      

      /* Retarget chains -------------------- */

      // TODO: Modify this loop so that joint state message is only filled out and published after all of the chains are retargeted. (Separate joint state messages for each chain are published right now)

      if(target_chains_.size() == 0)
      	return;
      
      auto retargeting_it = retargetings_.begin();
      for(auto chain_it = target_chains_.begin(); chain_it != target_chains_.end(); ++chain_it, ++retargeting_it)
      	{
      	  std::string chain_name = retargeting_it->name_;

	  
	  // Visualize untargeted target chain
      	  /* ROS_INFO( "Visualizing untargeted chain..."); */
	  /* unsigned int nr_target_joints = chain_it->getNrOfJoints(); */
      	  /* _JntArray target_angles = _JntArray(nr_target_joints); */
      	  /* for(unsigned int i = 0; i < nr_target_joints; ++i) */
      	  /*   target_angles(i) = 0.0; */

      	  /* visualizeKDLChain(*chain_it, target_angles, chain_name + "_untargeted", red_template_); */
	  	  	  
      	  // Get source chain pose array from humanoid message
      	  ROS_INFO( "Visualizing source chain... [ %s ]", chain_name.c_str());

	  _FrameArray source_frames;
	  
	  if(frameArrayFromHumanoid(retargeting_it->endpoints_["source_begin"],
				      retargeting_it->endpoints_["source_end"],
				      retargeting_it->endpoints_["source_parent"],
				      humanoid, source_frames))
	    {
	      ROS_WARN("Could not extract source chain from humanoid");
	      continue;
	    }

	  /**
	   * target chain is considered retargeted if it matches source chain after
	   * specified target-to-source transformation is applied
	   * Both chains are translated such that their base joints are located at
	   * the origin
	   */

	  krt::spatial::transform(source_frames, retargeting_it->target_to_source_);
	  krt::spatial::translateToOrigin(source_frames);
	  
	  visualizeFrameArray(krt::spatial::normalize(source_frames), chain_name + "_source", red_template_);
	  
	  ROS_INFO("Retargeting... [ %s ]", chain_name.c_str());
	  
	  krt::PoseSolverLsq retargeter(*chain_it);

	  if(!retargeter.update(source_frames, config_.ee_weight, 
				config_.step_threshold, 
				config_.max_iterations))
	    {
	      ROS_WARN("Retargeting failed. [ %s ]", chain_name.c_str());
	      continue;
	    }
	  ROS_INFO("Retargeting successful. [ %s ]", chain_name.c_str());
	  
	  _JntArray   retargeted_angles = retargeter.getTargetAngles();
	  _FrameArray retargeted_frames = retargeter.getTargetPose();
	  
	  visualizeFrameArray(krt::spatial::normalize(retargeted_frames), chain_name + "_retargeted", blue_template_);

	  /* Publish retargeted joint angles -------------------- */
	  
	  _JointStateMsg retargeted_joint_state;
	  
	  // TODO: Not sending angles for end effector joint. I should really just fix this and not minimize with end effector angle as an argument

	  for(unsigned int i = 0; i < chain_it->getNrOfJoints()-1; ++i)
	    {
	      _JointName joint_name = chain_it->getSegment(i).getJoint().getName();
	      
	      retargeted_joint_state.name.push_back(joint_name);
	      retargeted_joint_state.position.push_back(retargeted_angles(i));
	    }

	  multi_pub_.publish("joint_states", retargeted_joint_state);
	}
    }
  
  QUICKDEV_DECLARE_MESSAGE_CALLBACK( humanoidStatesCB, _HumanoidStateArrayMsg )
    {
      // get a lock to the cache
      QUICKDEV_TRY_LOCK_MUTEX( state_arrays_cache_ );
      // test the lock on the cache or return
      QUICKDEV_TRY_LOCK_OR_RETURN( state_arrays_cache_, "Dropping message [ %s ]", QUICKDEV_GET_MESSAGE_INST_NAME( msg ).c_str() );
      // if we got the lock, get a read/write reference to the cache's data
      auto & state_arrays_cache = state_arrays_cache_.get();

      state_arrays_cache.push_back( msg );
    }

};

#endif // KINEMATICRETARGETING_HUMANOIDRETARGETERNODE_H_
