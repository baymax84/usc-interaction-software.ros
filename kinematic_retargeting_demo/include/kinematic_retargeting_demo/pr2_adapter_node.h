/***************************************************************************
 *  include/kinematic_retargeting_demo/pr2_adapter_node.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dylan Foster (turtlecannon@gmail.com)
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
 *  * Neither the name of USC AUV nor the names of its
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


#ifndef USCAUV_KINEMATICRETARGETINGDEMO_PR2ADAPTER
#define USCAUV_KINEMATICRETARGETINGDEMO_PR2ADAPTER

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

/// cpp
#include <thread>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/multi_reconfigure.h>
#include <uscauv_common/simple_math.h>

#include <kinematic_retargeting_demo/RobotArmConfig.h>

typedef sensor_msgs::JointState _JointStateMsg;
typedef kinematic_retargeting_demo::RobotArmConfig _RobotArmConfig;

class JointFilterStorage
{
 public:
  std::vector<double> measurements_;
  unsigned int measurement_count_;
  
 JointFilterStorage(): measurement_count_(0)
    {
      measurements_.push_back(0);
    }
};

class JointFilter
{
 private:
  std::shared_ptr<JointFilterStorage> storage_ptr_;

  std::shared_ptr<JointFilterStorage> getStorage()
    {
      return storage_ptr_;
    }

  std::shared_ptr<JointFilterStorage const> getStorage() const
    {
      return std::const_pointer_cast<JointFilterStorage const>( storage_ptr_ );
    }
  
 public:
  
  JointFilter()
    {
      storage_ptr_ = std::make_shared<JointFilterStorage>();
    }

  void setQueueSize(unsigned int const & queue_size)
  {
    getStorage()->measurements_.resize( queue_size, 0 );
  }

  unsigned int getQueueSize() const
  {
    return getStorage()->measurements_.size();
  }
    
  double value() const 
  {
    double mean = 0;
    
    for( double const & elem : getStorage()->measurements_ )
	mean += elem;

    mean /= getQueueSize();
    
    return mean;
  }

  double var() const
  {
    double const mean = value();
    double var = 0;
    
    for( double const & elem : getStorage()->measurements_ )
      var += pow( uscauv::ring_distance( mean, elem, uscauv::TWO_PI ), 2 );

    var /= ( getQueueSize() );
    
    return var;
  }
  
  void update(double const & angle)
  {
    std::vector<double> & measurements = getStorage()->measurements_;
    
    for( unsigned int idx = 1; idx < measurements.size(); ++idx )
      {
	measurements[ idx ] = measurements[ idx - 1];
      }
    
    measurements[0] = angle;
    ++getStorage()->measurement_count_;
  }

  bool ready() const 
  {
    return ( getStorage()->measurement_count_ >= getStorage()->measurements_.size() );
  }
  
};

class ChainFilter
{
 private:
  typedef std::map<std::string, JointFilter> _JointMap;
  
  _JointMap joints_;

 private:

  double quality() const 
  {
    double max_var = -1;
    for( _JointMap::value_type const & elem : joints_ )
      {
	double const elem_var = elem.second.var();
	if( elem_var > max_var )
	  max_var = elem_var;
      }

    ROS_ASSERT( max_var > 0 );
    
    return max_var;
  }
  
 public:

  ChainFilter( std::vector<std::string> const & joint_names, unsigned int queue_size = 50 )
    {
      for( std::string const & name : joint_names )
	{
	  JointFilter joint = JointFilter();
	  joint.setQueueSize( queue_size );
	  joints_[ name ] = joint;
	}
    }

  void setQueueSize( unsigned int queue_size )
  {
    for( _JointMap::value_type & joint : joints_ )
      joint.second.setQueueSize( queue_size );
  }
  
  bool ready( double const & var_thresh ) const 
  {
    bool ready = joints_.size();
    if (!ready) return false;
    
    for( _JointMap::value_type const & elem : joints_ )
      {
	ready &= elem.second.ready();
      }
    if( !ready ) return false;

    ready &= (quality() <= var_thresh);
    
    return ready;    
  }

  void update( _JointStateMsg::ConstPtr const & msg )
  {
    if( msg->name.size() != msg->position.size() )
      {
	ROS_WARN("Invalid joint state message. Discarding...");
	return;
      }
    
    std::vector<std::string>::const_iterator name_it = msg->name.begin();
    std::vector<double>::const_iterator pos_it = msg->position.begin();
    for(; name_it != msg->name.end(); ++name_it, ++pos_it)
      {
	_JointMap::iterator joint_it = joints_.find( *name_it );
	if( joint_it != joints_.end() )
	  joint_it->second.update( *pos_it );
      }
  }
  
  trajectory_msgs::JointTrajectory generateTrajectory() const 
    {
      trajectory_msgs::JointTrajectory trajectory;
      trajectory_msgs::JointTrajectoryPoint point;

      for( _JointMap::value_type const & joint : joints_ )
      {
	trajectory.joint_names.push_back( joint.first );
	point.positions.push_back( joint.second.value() );
	point.velocities.push_back (0.0f);
	
      }
      point.time_from_start = ros::Duration( 0 );
      trajectory.points.push_back( point );
      
      trajectory.header.stamp = ros::Time::now();
      
      return trajectory;
    };

};

class RobotArm
{
 private:
  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> 
    _JointTrajectoryActionClient;

  std::shared_ptr<_JointTrajectoryActionClient> action_client_;
  
  ChainFilter joints_;
  _RobotArmConfig config_;
  std::mutex joints_mutex_;

  bool sent_;

 public:

 RobotArm(std::string path, std::vector<std::string> joint_names = std::vector<std::string>()) :
  joints_(joint_names), sent_(false)
  {
    // tell the action client that we want to spin a thread by default
    action_client_ = std::make_shared<_JointTrajectoryActionClient>(path, true); //"r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while (!action_client_->waitForServer(ros::Duration(5.0)))
      ROS_INFO("Robot arm action client waiting for server [ %s ].", path.c_str() );
    
    std::thread action_thread( &RobotArm::actionThread, this );
    action_thread.detach();    
  } 

  void updateConfig( _RobotArmConfig const & config )
  {
    std::unique_lock<std::mutex> lock( joints_mutex_ );

    config_ = config;
    joints_.setQueueSize( config_.sample_size );
  }

  void updateJoints(_JointStateMsg::ConstPtr const & msg)
  {
    std::unique_lock<std::mutex> lock( joints_mutex_ );
    joints_.update( msg ); 
  }

  void actionThread()
  {
    /// Completely arbitrary - Change if necessary
    ros::Rate loop_rate( 30 );

    ROS_INFO("Launched arm action thread." );
    
    while( ros::ok() )
      {
	{
	  std::unique_lock<std::mutex> lock( joints_mutex_ );

	  /* actionlib::SimpleClientGoalState state = action_client_->getState(); */
	  bool stable = joints_.ready( config_.var_ceiling );
	  
	  if( stable && !sent_ )
	    {
	      ROS_INFO("Sending new arm trajectory.");
	      
	      pr2_controllers_msgs::JointTrajectoryGoal goal;
	      goal.trajectory = joints_.generateTrajectory();
	      
	      action_client_->sendGoal(goal, boost::bind( &RobotArm::actionDoneCallback, this, _1, _2 ) /* _JointTrajectoryActionClient::SimpleDoneCallback() */,
				       _JointTrajectoryActionClient::SimpleActiveCallback(),
				       boost::bind( &RobotArm::actionFeedbackCallback, this, _1 ) );
	      sent_ = true;
	    }  
	  /* else if ( !stable && !state.isDone() ) */
	  /*   { */
	  /*     ROS_INFO("Pose changed. Cancelling trajectory..."); */
	      
	  /*     action_client_->cancelGoal(); */
	  /*   } */
	  
	}
	/// Released the lock, now sleep
	loop_rate.sleep();
      }
  }
 public:
  
  void actionDoneCallback( actionlib::SimpleClientGoalState const & state, pr2_controllers_msgs::JointTrajectoryResultConstPtr const & result )
  {
    std::unique_lock<std::mutex> lock( joints_mutex_ );
    sent_ = false;
    return;
  }

  void actionFeedbackCallback( pr2_controllers_msgs::JointTrajectoryFeedbackConstPtr const & feedback )
  {
    std::unique_lock<std::mutex> lock( joints_mutex_ );
    if( ! joints_.ready( config_.var_ceiling ) )
      {
	ROS_INFO("Pose changed. Cancelling trajectory...");
	action_client_->cancelGoal();
	sent_ = false;
      }
    return;
  }
  
};

class Pr2AdapterNode: public BaseNode, public MultiReconfigure
{
  std::shared_ptr<RobotArm> right_arm_, left_arm_;
  
 public:
 Pr2AdapterNode(): BaseNode("Pr2Adapter")
    {
    }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    std::vector<std::string> right_arm_names = 
      {
	"r_shoulder_lift_link",
	"r_shoulder_pan_link",
	"r_upper_arm_link",
	"r_upper_arm_roll_link",
	"r_elbow_flex_link",
	"r_forearm_link",
	"r_forearm_roll_link"
      };

    std::vector<std::string> left_arm_names = 
      {
	"l_shoulder_lift_link",
	"l_shoulder_pan_link",
	"l_upper_arm_link",
	"l_upper_arm_roll_link",
	"l_elbow_flex_link",
	"l_forearm_link",
	"l_forearm_roll_link"
      };
    
    right_arm_ = std::make_shared<RobotArm>( "r_arm_controller/joint_trajectory_action", right_arm_names );
    left_arm_ = std::make_shared<RobotArm>( "l_arm_controller/joint_trajectory_action", left_arm_names );

    addReconfigureServer<_RobotArmConfig>("right_arm", &RobotArm::updateConfig, right_arm_.get() );
    addReconfigureServer<_RobotArmConfig>("left_arm", &RobotArm::updateConfig, left_arm_.get() );
  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {

  }

};

#endif // USCAUV_KINEMATICRETARGETINGDEMO_PR2ADAPTER
