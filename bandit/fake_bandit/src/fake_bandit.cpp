#include "ros/ros.h"
#include "ros/console.h"
#include <bandit_msgs/Params.h>
#include <bandit_msgs/JointArray.h>
#include <sensor_msgs/JointState.h>

//typedef bandit_msgs::Params::Reponse Reponse;

bandit_msgs::Params::Response param_res;
//Reponse param_res;

//#define DTOR( a ) a * M_PI / 180.0
//#define RTOD( a ) a * 180.0 / M_PI

ros::Publisher joint_state_publisher;
std::vector<std::string> joint_names;
std::vector<double> joint_pos;
std::vector<double> zeroes;


bool param(bandit_msgs::Params::Request& inRequest, bandit_msgs::Params::Response& inResponse)
{
  	inResponse = param_res;
  	return true;
}


void add_param( int id, std::string name, float min, float max, float pos )
{
  	param_res.id.push_back(id);
	param_res.name.push_back(name);
	param_res.min.push_back(min);
	param_res.max.push_back(max);
	param_res.pos.push_back(pos);
  	joint_pos.push_back(pos);
  	joint_names.push_back(name);
  	zeroes.push_back(0);
}

void jointCB( const bandit_msgs::JointConstPtr& j )
{
  	joint_pos[j->id] = j->angle;
}

void jointarrayCB( const bandit_msgs::JointArrayConstPtr& j )
{
	for( unsigned int i = 0; i < j->joints.size(); i++ )
	{
		int id = j->joints[i].id;
		//double angle = j->joints[i].angle;
		//joint_pos[id] = angle;
		joint_pos[id] = j->joints[i].angle;
	}
}

void targetCB( const sensor_msgs::JointStateConstPtr& j )
{
	for( unsigned int i = 0; i < j->name.size(); i++ )
	{
		// find id of joint
		int id = -1;
		for( unsigned int jj = 0; jj < param_res.name.size(); jj++ )
		{
			if( param_res.name[jj] == j->name[i] )
			{
				id = param_res.id[jj];
				break;
			}
		}

		if( id < 0 ) break;

		double angle = j->position[i];
		joint_pos[id] = angle;
	}
}

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "fake_bandit" );
	ros::NodeHandle n;
  	ros::Subscriber joints_sub = n.subscribe("joint_cmd", 1, jointarrayCB );
  	ros::Subscriber target_sub = n.subscribe("target_joints", 1, targetCB );
  	joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1000);


	// head  
  	//add_param( 0, "bandit_head_pan_joint", -90, 90, 0 );
  	//add_param( 1, "bandit_head_tilt_joint", -15, 15, 0 );
  	add_param( 0, "head_pan_joint", -90, 90, 0 );
  	add_param( 1, "head_tilt_joint", -15, 15, 0 );

	// left arm
  	add_param( 2, "left_torso_shoulder_mounting_joint", -180, 180, 0);
  	add_param( 3, "left_shoulder_mounting_shoulder_joint", -20, 180, 0);
  	add_param( 4, "left_shoulder_bicep_joint", -90, 90, 0);
  	add_param( 5, "left_bicep_forearm_joint", 0, 110, 0);
  	add_param( 6, "left_forearm_wrist_joint", -90, 90, 0);
  	add_param( 7, "left_wrist_hand_joint", -90, 90, 0);
  	add_param( 8, "left_hand_thumb_joint", -90, 90, 0);

	// right arm
  	add_param( 9, "right_torso_shoulder_mounting_joint", -90, 180, 0);
  	add_param( 10, "right_shoulder_mounting_shoulder_joint", -20, 180, 0);
  	add_param( 11, "right_shoulder_bicep_joint", -90, 90, 0);
  	add_param( 12, "right_bicep_forearm_joint", 0, 110, 0);
  	add_param( 13, "right_forearm_wrist_joint", -90, 90, 0);
  	add_param( 14, "right_wrist_hand_joint", -90, 90, 0);
  	add_param( 15, "right_hand_thumb_joint", -90, 90, 0);
  
  
  	//add_param( 16, "eyebrows_joint", -90, 90, 0);
  	add_param( 16, "mouth_top_joint", 0, 30, 0);
  	add_param( 17, "mouth_bottom_joint", 0, -30, 0);
  	
  	
  	// testing
  	//add_param(18, "
  	add_param(18, "head_left_brow_joint", -90, 90, 0);
  	add_param(19, "head_right_brow_joint", -90, 90, 0);
  	
  	// super testing
  	add_param(20, "pupil_left_joint", -90, 90, 0);
  	add_param(21, "pupil_right_joint", -90, 90, 0);
  	
  	
  	// end testing

  	ros::ServiceServer service = n.advertiseService("params", param);
  	ros::Rate loop_rate(50);
 
  	sensor_msgs::JointState js;

  	while( n.ok() )
  	{
 
    	//js.header.frame_id="/bandit_torso_link";
    	js.header.frame_id="/stop_link";
    	
    	js.header.stamp = ros::Time::now();
    	js.name = joint_names;
    	js.position = joint_pos;
    	js.velocity = zeroes;
    	js.effort = zeroes;    
 

    	for( unsigned int i = 0; i < js.name.size(); i++ )
    	{
    		/*
      		if( js.name[i] == std::string( "eyebrows_joint" ) )
      		{
        		js.name[i] = std::string("bandit_head_left_brow_joint");
        
        		js.name.push_back( "bandit_head_right_brow_joint");
        		js.position.push_back(js.position[i]);
        		js.velocity.push_back(js.velocity[i]);
        		js.effort.push_back(js.effort[i]);
      		}
      		*/
    	}

    	joint_state_publisher.publish(js);
  
    	//ROS_INFO( "publish" );

	 	ros::spinOnce();
    	loop_rate.sleep();
  	}

	return 0;
}

