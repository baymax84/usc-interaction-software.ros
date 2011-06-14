/*******************************************************************************
 *
 *      bandit_node
 * 
 *      Copyright (c) 2011, edward
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "interaction-ros-pkg" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef BANDIT_NODE_H_
#define BANDIT_NODE_H_

#include <ros/ros.h>
#include <bandit_msgs/JointArray.h>
#include <bandit_msgs/Params.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <sstream>
#include <bandit/bandit.h>
#include <yaml-cpp/yaml.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <string>
#include <vector>

#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <interaction_common/math.h>

template<class _DataType = int>
struct JointPID
{
	_DataType id, p, i, d, i_min, i_max, e_min, offset;

	JointPID() :
		id( -1 ), p( 100 ), i( 0 ), d( 0 ), i_min( -4000 ), i_max( 4000 ), e_min( 50 ), offset( 0 )
	{
		//
	}
};

//This creates a data structure for mappings in yaml file
struct JointCalibration
{
	int id, direction;
	float true_zero, offset, max_angle, min_angle;
};

typedef JointPID<int> _JointPID;

typedef std::vector<JointCalibration> _JointCalibrationVector;

void operator >>( const YAML::Node & node, _JointPID & joint_pid )
{
	node["id"] >> joint_pid.id;
	node["p"] >> joint_pid.p;
	node["i"] >> joint_pid.i;
	node["d"] >> joint_pid.d;
	node["i_min"] >> joint_pid.i_min;
	node["i_max"] >> joint_pid.i_max;
	node["e_min"] >> joint_pid.e_min;
	node["offset"] >> joint_pid.offset;
}

//This overrides the >> operator to insert mappings into joint
void operator >>( const YAML::Node& node, JointCalibration& joint )
{
	node["JointID"] >> joint.id;
	node["Direction"] >> joint.direction;
	node["TrueZero"] >> joint.true_zero;
	node["Offset"] >> joint.offset;
	node["MaxAngle"] >> joint.max_angle;
	node["MinAngle"] >> joint.min_angle;
}

class BanditNode
{
public:

	typedef _JointCalibrationVector::iterator _JointCalibrationIterator;
	typedef _JointCalibrationVector::const_iterator _JointCalibrationConstIterator;

	typedef YAML::Iterator _YAMLNodeIterator;
	typedef YAML::Iterator _YAMLNodeConstIterator;

	bandit::Bandit bandit_driver_;
	std::string pid_config_uri_;
	std::string calibration_filename_;
	bandit_msgs::Params::Response param_response_;
	diagnostic_updater::Updater diagnostic_;
	ros::NodeHandle nh_priv_;

	ros::Subscriber joint_sub_;
	ros::Subscriber target_sub_;
	ros::ServiceServer service_;

	diagnostic_updater::DiagnosedPublisher<sensor_msgs::JointState> joints_pub_;

	BanditNode( ros::NodeHandle & nh, double desired_freq = 5.0 ) :
				nh_priv_( "~" ),
				joints_pub_( nh.advertise<sensor_msgs::JointState> ( "joint_states", 1000 ), diagnostic_, diagnostic_updater::FrequencyStatusParam( &desired_freq, &desired_freq, 0.5 ),
						diagnostic_updater::TimeStampStatusParam() )
	{
		// Retrieve port from parameter server
		std::string port;

		nh_priv_.param( "port", port, std::string( "/dev/ttyUSB0" ) );
		nh_priv_.param( "pid_config_uri", pid_config_uri_, std::string( "" ) );
		nh_priv_.param( "calib_uri", calibration_filename_, std::string( "" ) );

		ROS_INFO( "creating services and subscribing to topics" );

		// Now that things are supposeldy up and running, subscribe to
		// joint messages
		joint_sub_ = nh.subscribe( "joint_cmd", 1, &BanditNode::jointCB, this );
		target_sub_ = nh.subscribe( "target_joints", 1, &BanditNode::targetCB, this );
		service_ = nh.advertiseService( "params", &BanditNode::paramCB, this );

		std::ifstream fin;

		//ros::Publisher joints_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

		// joint offset parameters for Bandit #1 (RAM-2010Apr30)
		//nh.param( "home", homestring, std::string("-3,10,-62,-74,-28,62,-11,0.5,0.6,52,80,26,-58,12,0.5,0.5,0.2,0.25,0.25,"));
		//nh.param( "direction", dirsstring, std::string("-1,-1,1,1,1,-1,1,1,1,-1,-1,-1,1,-1,-1,1,1,1,-1,"));

		// do setup
		try
		{

			ROS_INFO( "connecting to bandit on port: [%s]...", port.c_str() );

			// Open the port
			bandit_driver_.openPort( port.c_str() );

			ROS_INFO( "Success." );

			// This 19 shouldn't be hard coded.  Will improve API for this
			// later.  Also, should set this up as a Service API to tweak
			// gains online as well.

			// load and parse calibration info
			fin.open( calibration_filename_.c_str() );
			if ( fin.fail() )
			{
				ROS_WARN( "Failed to find calibration file [%s], running uncalibrated", calibration_filename_.c_str() );
				bandit_driver_.useJointLimits( false );
			}
			else
			{
				YAML::Parser parser( fin );
				YAML::Node doc;
				parser.GetNextDocument( doc );

				ROS_INFO( "locating top-level in calibration file" );

				const YAML::Node & joint_calibrations = doc["JointCalibrations"];
				_YAMLNodeConstIterator joint_it = joint_calibrations.begin();

				for ( size_t i = 0; joint_it != joint_calibrations.end(); ++joint_it, ++i )
				{
					JointCalibration joint_calibration;
					*joint_it >> joint_calibration;
					int id = joint_calibration.id;
					printf( " id = %d \n", id );
					// These default PID gains and offsets should be read from a config file
					if ( bandit_driver_.getJointType( id ) == smartservo::SMART_SERVO )
					{
						bandit_driver_.setJointPIDConfig( id, 100, 0, 0, -4000, 4001, 50, 0 );
					}

					//set joint offsets and directions
					bandit_driver_.setJointDirection( id, joint_calibration.direction );
					//bandit_driver_.setJointOffset(i, DTOR(home[i]));
					if ( bandit_driver_.getJointType( id ) == smartservo::SMART_SERVO ) bandit_driver_.setJointOffset( id, degToRad( joint_calibration.true_zero ) );
					//g_bandit_.setJointOffset(i, DTOR(home[i]));
					else bandit_driver_.setJointOffset( id, joint_calibration.true_zero );
					//bandit_driver_.setJointOffset(i, home[i]);
				}

				fin.close();
			}

			//populate service response message
			const size_t num_joints = bandit_driver_.getNumJoints();
			param_response_.id.reserve( num_joints );
			param_response_.name.reserve( num_joints );
			param_response_.min.reserve( num_joints );
			param_response_.max.reserve( num_joints );
			param_response_.pos.reserve( num_joints );

			for ( size_t i = 0; i < bandit_driver_.getNumJoints(); ++i )
			{
				param_response_.id.push_back( i );
				param_response_.name.push_back( bandit_driver_.getJointRosName( i ) );
				param_response_.min.push_back( radToDeg( bandit_driver_.getJointMin( i ) ) );
				param_response_.max.push_back( radToDeg( bandit_driver_.getJointMax( i ) ) );
				param_response_.pos.push_back( radToDeg( bandit_driver_.getJointPos( i ) ) );
			}

			// load and parse PID info
			fin.open( pid_config_uri_.c_str() );
			if ( !fin.good() )
			{
				ROS_WARN( "PID config file not found; using default PID values" );
			}
			else
			{
				YAML::Parser parser( fin );
				YAML::Node doc;

				parser.GetNextDocument( doc );

				ROS_INFO( "locating top-level in pid configuration" );

				const YAML::Node & joint_pids = doc["JointPIDs"];

				_YAMLNodeConstIterator joint_pid_it = joint_pids.begin();
				for ( size_t i = 0; joint_pid_it != joint_pids.end(); ++joint_pid_it, ++i )
				{
					if ( bandit_driver_.getJointType( i ) == smartservo::SMART_SERVO )
					{
						_JointPID joint_pid;
						*joint_pid_it >> joint_pid;
						bandit_driver_.setJointPIDConfig( joint_pid.id, joint_pid.p, joint_pid.i, joint_pid.d, joint_pid.i_min, joint_pid.i_max, joint_pid.e_min, joint_pid.offset );
					}
				}
			}
			fin.close();

			// Synchronize PID gains
			// we set the first time back in time to guarantee it gets hit
			// ^wtfisthisshit.jpg
			ros::Time try_again = ros::Time::now() - ros::Duration( .1 );
			int tries = 0;
			do
			{
				if ( try_again < ros::Time::now() )
				{
					bandit_driver_.sendAllPIDConfigs();
					try_again = try_again + ros::Duration( 1.0 );
					if ( ++tries % 10 == 0 ) ROS_ERROR( "Failed to configure PID settings after %d tries", tries );
				}

				bandit_driver_.processIO( 10000 );
				bandit_driver_.processPackets();

				ros::spinOnce();

			}
			while ( !bandit_driver_.checkAllPIDConfigs() );

			ROS_INFO( "All PID settings configured successfully" );

			// Push out initial state
			for ( size_t i = 0; i < bandit_driver_.getNumJoints(); ++i )
			{
				if ( i == 3 || i == 10 ) bandit_driver_.setJointPos( i, degToRad( 15.0 ) );
				else bandit_driver_.setJointPos( i, 0.0f );

				ros::spinOnce();
			}

			ROS_INFO( "Sending initial joint positions" );

			// Send bandit position commands:
			bandit_driver_.sendAllJointPos();
			ros::spinOnce();

			ROS_INFO( "initializing IO processing loop" );
			// This callback gets called whenever processPendingMessages
			// receives a valid state update from bandit
			bandit_driver_.registerStateCB( boost::bind( &BanditNode::stateCB, this, boost::ref( joints_pub_ ) ) );
			ROS_INFO( "registered state callback" );
			ros::spinOnce();

		}
		catch ( bandit::BanditException& e )
		{
			ROS_ERROR( "Caught bandit exception: %s\n", e.what() );
		}
	}

	bool paramCB( bandit_msgs::Params::Request &req, bandit_msgs::Params::Response &res )
	{
		res = param_response_;
		return true;
	}

	// This callback is invoked when we get a new joint command
	void jointCB( const bandit_msgs::JointArrayConstPtr& joint_array )
	{
		// Iterate through all joints in Joint Array
		std::vector<bandit_msgs::Joint>::const_iterator joint_it = joint_array->joints.begin();
		for ( size_t i = 0; joint_it != joint_array->joints.end(); ++joint_it, ++i )
		{
			// Set the joint position
			ROS_INFO( "setting joint %zu to angle: %f\n", joint_it->id, radToDeg( joint_it->angle ) );

			// Set the joint position; if this index doesn't exist, Bandit will throw an error so make sure to account for this
			bandit_driver_.setJointPos( joint_it->id, joint_it->angle );
		}
		// Push out positions to bandit

		bandit_driver_.sendAllJointPos();
	}

	void targetCB( const sensor_msgs::JointStateConstPtr & target_joint_state )
	{
		std::vector<std::string>::const_iterator joint_name_it = target_joint_state->name.begin();
		std::vector<double>::const_iterator joint_position_it = target_joint_state->position.begin();
		for ( ; joint_name_it != target_joint_state->name.end(); ++joint_name_it, ++joint_position_it )
		{
			for ( size_t joint_index = 0; joint_index < bandit_driver_.getNumJoints(); ++joint_index )
			{
				if ( bandit_driver_.getJointRosName( joint_index ) == *joint_name_it )
				{
					bandit_driver_.setJointPos( joint_index, *joint_position_it );
				}
			}
		}

		bandit_driver_.sendAllJointPos();
	}

	// This callback is invoked when we get new state from bandit
	void stateCB( diagnostic_updater::DiagnosedPublisher<sensor_msgs::JointState>& joint_state_pub )
	{
		// let's assume we're not going to change the number of joints or their name mappings after initialization
		const static size_t num_joints = bandit_driver_.getNumJoints();
		// lookup the eyebrow joint to prevent doing this for every joint every function call
		const static size_t eyebrows_joint_index = bandit_driver_.getJointIndexByROSName( "eyebrows_joint" );
		sensor_msgs::JointState joint_state;

		// reserve memory now to make push_back operations less costly
		joint_state.name.reserve( num_joints );
		joint_state.position.reserve( num_joints );
		joint_state.velocity.reserve( num_joints );
		joint_state.effort.reserve( num_joints );

		joint_state.header.stamp = ros::Time::now();
		joint_state.header.frame_id = "bandit_torso_link";

		// For every joint
		for ( size_t i = 0; i < num_joints; ++i )
		{
			if ( i == eyebrows_joint_index )
			{
				// the eyebrows are really two joints as far as robot_state_publisher is concerned

				joint_state.name.push_back( std::string( "bandit_head_left_brow_joint" ) );
				joint_state.position.push_back( -3 * bandit_driver_.getJointPos( i ) );
				joint_state.velocity.push_back( 0 );
				joint_state.effort.push_back( 0 );
				joint_state.name.push_back( std::string( "bandit_head_right_brow_joint" ) );
				joint_state.position.push_back( -3 * bandit_driver_.getJointPos( i ) );
				joint_state.velocity.push_back( 0 );
				joint_state.effort.push_back( 0 );

			}
			else
			{
				joint_state.name.push_back( bandit_driver_.getJointRosName( i ) );
				if ( i > 16 ) joint_state.position.push_back( 2 * bandit_driver_.getJointPos( i ) );
				else
				{
					static double angle;
					angle = bandit_driver_.getJointPos( i );
					while( angle >  M_PI )
					{
						angle -= 2* M_PI;
					}
					while( angle < -M_PI )
					{
						angle += 2 * M_PI;
					}
					joint_state.position.push_back( angle );
				}

				joint_state.velocity.push_back( 0 );
				joint_state.effort.push_back( 0 );
			}
		}

		//ROS_INFO( "publishing..." );

		// Publish to other nodes
		joint_state_pub.publish( joint_state );
	}

	void spin()
	{
		// update loop
		while ( ros::ok() )
		{
			try
			{
				diagnostic_.update();

				// Process any pending messages from bandit
				//ROS_INFO( "processIO: %ld", loop_rate.expectedCycleTime().toNSec());

				bandit_driver_.processIO( 5000 );
				//bandit_driver_.processIO(loop_rate.expectedCycleTime().toNSec());
				//ROS_INFO( "processPackets" );
				bandit_driver_.processPackets();
				ros::spinOnce();
			}
			catch ( bandit::BanditException& e )
			{
				ROS_ERROR( "Caught bandit exception: %s\n", e.what() );
			}
		}
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "bandit_driver" );
	ros::NodeHandle nh;

	printf( "%f %f\n", degToRad( 90.0 ), radToDeg( 1.570796 ) );

	BanditNode bandit_node( nh );
	bandit_node.spin();

	return 0;
}

#endif /* BANDIT_NODE_H_ */
