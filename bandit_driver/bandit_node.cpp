#include <ros/ros.h>
#include <bandit_msgs/JointArray.h>
#include <bandit_msgs/Params.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <bandit/bandit.h>
#include <yaml-cpp/yaml.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <string>

#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/diagnostic_updater.h>

bandit::Bandit g_bandit;
std::string pid_config_uri;
std::string calibration_filename;

#define DTOR( a ) a * M_PI / 180.0
#define RTOD( a ) a * 180.0 / M_PI

bandit_msgs::Params::Response param_res;

struct JointPID
{
	int id, p, i, d, i_min, i_max, e_min, offset;

	JointPID ()
	{
		id = -1;
		p = 100;
		i = 0;
		d = 0;
		i_min = -4000;
		i_max = 4000;
		e_min = 50;
		offset = 0;
	}
};

void operator >> ( const YAML::Node & node, JointPID & j )
{
	node["id"] >> j.id;
	node["p"] >> j.p;
	node["i"] >> j.i;
	node["d"] >> j.d;
	node["i_min"] >> j.i_min;
	node["i_max"] >> j.i_max;
	node["e_min"] >> j.e_min;
	node["offset"] >> j.offset;
}

bool param( bandit_msgs::Params::Request &req, bandit_msgs::Params::Response &res )
{
	res = param_res;
	return true;
}

// This callback is invoked when we get a new joint command
void jointCB( const bandit_msgs::JointArrayConstPtr& j )
{
	// Iterate through all joints in Joint Array
	for ( std::vector<bandit_msgs::Joint>::const_iterator joint_iter = j->joints.begin(); joint_iter != j->joints.end(); joint_iter++ )
	{
		// Set the joint position
		ROS_INFO( "j->angle: %f\n", RTOD(joint_iter->angle) );


		// Set the joint position
		g_bandit.setJointPos( joint_iter->id, joint_iter->angle );
	}
	// Push out positions to bandit

	g_bandit.sendAllJointPos();
}

void targetCB( const sensor_msgs::JointStateConstPtr &t )
{
	for ( unsigned int i = 0; i < t->name.size(); i++ )
	{
		for ( int j = 0; j < 19; j++ )
		{
			if ( g_bandit.getJointRosName( j ) == t->name[i] )
			{
				g_bandit.setJointPos( j, t->position[i] );
			}
		}
	}

	g_bandit.sendAllJointPos();
}

// This callback is invoked when we get new state from bandit
void stateCB( diagnostic_updater::DiagnosedPublisher<sensor_msgs::JointState>& joint_pub )
{
	sensor_msgs::JointState js;

	js.header.stamp = ros::Time::now();
	js.header.frame_id = "bandit_torso_link";


	// For every joint
	for ( int i = 0; i < 19; i++ )
	{
		if ( g_bandit.getJointRosName( i ) == std::string( "eyebrows_joint" ) )
		{
			// the eyebrows are really two joints as far as robot_state_publisher is concerned

			js.name.push_back( std::string( "bandit_head_left_brow_joint" ) );
			js.position.push_back( -3 * g_bandit.getJointPos( i ) );
			js.velocity.push_back( 0 );
			js.effort.push_back( 0 );
			js.name.push_back( std::string( "bandit_head_right_brow_joint" ) );
			js.position.push_back( -3 * g_bandit.getJointPos( i ) );
			js.velocity.push_back( 0 );
			js.effort.push_back( 0 );

		}
		else
		{
			js.name.push_back( g_bandit.getJointRosName( i ) );
			if ( i > 16 ) js.position.push_back( 2 * g_bandit.getJointPos( i ) );
			else js.position.push_back( g_bandit.getJointPos( i ) );

			js.velocity.push_back( 0 );
			js.effort.push_back( 0 );
		}
	}


	//ROS_INFO( "publishing..." );

	// Publish to other nodes
	joint_pub.publish( js );
}

//This creates a data structure for mappings in yaml file
struct Joint_Calibrations
{
	int id, direction;
	float truezero, offset, maxAngle, minAngle;
};

//This overrides the >> operator to insert mappings into joint
void operator >>( const YAML::Node& node, Joint_Calibrations& joint )
{
	node["JointID"] >> joint.id;
	node["Direction"] >> joint.direction;
	node["TrueZero"] >> joint.truezero;
	node["Offset"] >> joint.offset;
	node["MaxAngle"] >> joint.maxAngle;
	node["MinAngle"] >> joint.minAngle;
}

int main( int argc, char** argv )
{
	// Tells a joint to move in the same or opposite direction
	// of the normal Bandit movement
	int direction[19];


	// Home positions of the joints for Bandit
	//double home[19];
	double j_cal[19];

	ros::init( argc, argv, "bandit_driver" );
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv( "~" );
	// Retrieve port from parameter server
	std::string port;

	nh_priv.param( "port", port, std::string( "/dev/ttyUSB0" ) );
	nh_priv.param( "pid_config_uri", pid_config_uri, std::string( "" ) );
	nh_priv.param( "calib_uri", calibration_filename, std::string( "" ) );

	std::ifstream fin;

	double desired_freq = 5.0;

	diagnostic_updater::Updater diagnostic;
	diagnostic_updater::DiagnosedPublisher<sensor_msgs::JointState> joints_pub( nh.advertise<sensor_msgs::JointState> ( "joint_states", 1000 ), diagnostic, diagnostic_updater::FrequencyStatusParam(
			&desired_freq, &desired_freq, 0.5 ), diagnostic_updater::TimeStampStatusParam() );


	//ros::Publisher joints_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

	std::string homestring, dirsstring;


	// joint offset parameters for Bandit #1 (RAM-2010Apr30)
	//nh.param( "home", homestring, std::string("-3,10,-62,-74,-28,62,-11,0.5,0.6,52,80,26,-58,12,0.5,0.5,0.2,0.25,0.25,"));
	//nh.param( "direction", dirsstring, std::string("-1,-1,1,1,1,-1,1,1,1,-1,-1,-1,1,-1,-1,1,1,1,-1,"));

	for ( int i = 0; i < 19; i++ )
	{
		direction[i] = 1;
		//home[i] = 0;
		j_cal[i] = 0;
	}
	
	fin.open( calibration_filename.c_str() );
	if ( fin.fail() )
	{
		ROS_WARN( "Failure to find calibration file [%s], running uncalibrated", calibration_filename.c_str() );
		g_bandit.useJointLimits( false );
	}
	else
	{
		YAML::Parser parser( fin );
		YAML::Node doc;
		parser.GetNextDocument( doc );
		Joint_Calibrations joint;

		for ( unsigned k = 0; k < doc.size(); k++ )
		{
			doc[k] >> joint;
			j_cal[k] = joint.truezero;
			ROS_INFO( "joint(%d): %0.2f", k, joint.truezero );
			direction[k] = joint.direction;
		}
		g_bandit.useJointLimits( true );
	}
	fin.close();

	std::string::size_type i = 0;
	std::string::size_type j = dirsstring.find( ',' );
	int ii = 0;
	
	ROS_INFO( "direction: " );
	while ( j != std::string::npos )
	{
		std::string s = dirsstring.substr( i, j - i );
		direction[ii] = atoi( s.c_str() );
		ROS_INFO( "%d:%s:%d", ii, s.c_str(), direction[ii] );
		++ii;
		i = ++j;
		j = dirsstring.find( ',', j );
	}

	try
	{
		// This callback gets called whenever processPendingMessages
		// receives a valid state update from bandit
		g_bandit.registerStateCB( boost::bind( &stateCB, boost::ref( joints_pub ) ) );

		ROS_INFO( "connecting to bandit on port: [%s]\n", port.c_str() );


		// Open the port
		g_bandit.openPort( port.c_str() );


		// This 19 shouldn't be hard coded.  Will improve API for this
		// later.  Also, should set this up as a Service API to tweak
		// gains online as well.
		for ( int i = 0; i < 19; i++ )
		{
			// These default PID gains and offsets should be read from a config file
			if ( g_bandit.getJointType( i ) == smartservo::SMART_SERVO )
			{
				g_bandit.setJointPIDConfig( i, 100, 0, 0, -4000, 4001, 50, 0 );
			}

			//set joint offsets and directions
			g_bandit.setJointDirection( i, direction[i] );
			//g_bandit.setJointOffset(i, DTOR(home[i]));
			if ( g_bandit.getJointType( i ) == smartservo::SMART_SERVO ) g_bandit.setJointOffset( i, DTOR(j_cal[i]));
			//g_bandit.setJointOffset(i, DTOR(home[i]));
			else g_bandit.setJointOffset( i, j_cal[i] );
			//g_bandit.setJointOffset(i, home[i]);

			//populate service response message
			param_res.id.push_back( i );
			param_res.name.push_back( g_bandit.getJointRosName( i ) );
			param_res.min.push_back( RTOD(g_bandit.getJointMin(i)) );
			param_res.max.push_back( RTOD(g_bandit.getJointMax(i)) );
			param_res.pos.push_back( RTOD(g_bandit.getJointPos(i)) );
		}

		fin.open( pid_config_uri.c_str() );
		if ( !fin.good() )
		{
			ROS_WARN( "PID config file not found; using default PID values" );
		}
		else
		{
			YAML::Parser parser( fin );
			YAML::Node doc;

			parser.GetNextDocument( doc );

			const YAML::Node & jointPIDs = doc["JointPIDs"];

			for ( unsigned int i = 0; i < jointPIDs.size(); i++ )
			{
				if ( g_bandit.getJointType( i ) == smartservo::SMART_SERVO )
				{
					JointPID temp;
					jointPIDs[i] >> temp;
					g_bandit.setJointPIDConfig( temp.id, temp.p, temp.i, temp.d, temp.i_min, temp.i_max, temp.e_min, temp.offset );
				}
			}
		}
		fin.close();

		// Synchronize PID gains
		// we set the first time back in time to guarantee it gets hit
		ros::Time try_again = ros::Time::now() - ros::Duration( .1 );
		int tries = 0;
		do
		{
			if ( try_again < ros::Time::now() )
			{
				g_bandit.sendAllPIDConfigs();
				try_again = try_again + ros::Duration( 1.0 );
				if ( ++tries % 10 == 0 ) ROS_ERROR( "Failed to configure PID settings after %d tries", tries );
			}

			g_bandit.processIO( 10000 );
			g_bandit.processPackets();

		}
		while ( !g_bandit.checkAllPIDConfigs() );

		ROS_INFO( "All PID settings configured successfully" );


		// Push out initial state
		for ( int i = 0; i < 19; i++ )
		{
			g_bandit.setJointPos( i, 0.0f );
		}

		// Send bandit position commands:
		g_bandit.sendAllJointPos();


		// Now that things are supposeldy up and running, subscribe to
		// joint messages
		ros::Subscriber joint_sub = nh.subscribe( "joint_cmd", 1, jointCB );
		ros::Subscriber target_sub = nh.subscribe( "target_joints", 1, targetCB );
		ros::ServiceServer service = nh.advertiseService( "params", param );

		ros::Rate loop_rate( 10 );

		while ( ros::ok() )
		{
			diagnostic.update();


			// Process any pending messages from bandit
			//ROS_INFO( "processIO: %ld", loop_rate.expectedCycleTime().toNSec());

			g_bandit.processIO( 5000 );
			//g_bandit.processIO(loop_rate.expectedCycleTime().toNSec());
			//ROS_INFO( "processPackets" );
			g_bandit.processPackets();
			ros::spinOnce();
		}

	}
	catch ( bandit::BanditException& e )
	{
		ROS_FATAL( "Caught bandit exception: %s\n", e.what() );
	}

	return 0;
}
