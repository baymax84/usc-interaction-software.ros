#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Publisher joint_pub_;

void joint_cb( const sensor_msgs::JointStateConstPtr& joint )
{
	sensor_msgs::JointState output;
	output.header = joint->header;

	for( unsigned i = 0; i < joint->name.size(); i++ )
	{
		// for each joint

		// convert name to new name
		if( joint->name[i] == "elbow_left_roll" ) 
		{
			output.name.push_back("left_bicep_forearm_joint");
			output.position.push_back(-joint->position[i]);
			output.velocity.push_back(0.0);
			output.effort.push_back(0.0);
		}
		if( joint->name[i] == "shoulder_left_roll" ) 
		{
			output.name.push_back("left_shoulder_mounting_shoulder_joint");
			output.position.push_back(joint->position[i]);
			output.velocity.push_back(0.0);
			output.effort.push_back(0.0);
		}
		if( joint->name[i] == "shoulder_left_pitch" ) 
		{
			output.name.push_back("left_torso_shoulder_mounting_joint");
			output.position.push_back(joint->position[i]);
			output.velocity.push_back(0.0);
			output.effort.push_back(0.0);
		}
		if( joint->name[i] == "shoulder_left_yaw" ) 
		{
			output.name.push_back("left_shoulder_bicep_joint");
			output.position.push_back(joint->position[i] - M_PI/2.);
			output.velocity.push_back(0.0);
			output.effort.push_back(0.0);
		}
		if( joint->name[i] == "elbow_right_roll" ) 
		{
			output.name.push_back("right_bicep_forearm_joint");
			output.position.push_back(joint->position[i]);
			output.velocity.push_back(0.0);
			output.effort.push_back(0.0);
		}
		if( joint->name[i] == "shoulder_right_roll" ) 
		{
			output.name.push_back("right_shoulder_mounting_shoulder_joint");
			output.position.push_back(-joint->position[i]);
			output.velocity.push_back(0.0);
			output.effort.push_back(0.0);
		}
		if( joint->name[i] == "shoulder_right_pitch" ) 
		{
			output.name.push_back("right_torso_shoulder_mounting_joint");
			output.position.push_back(-joint->position[i]);
			output.velocity.push_back(0.0);
			output.effort.push_back(0.0);
		}
		if( joint->name[i] == "shoulder_right_yaw" ) 
		{
			output.name.push_back("right_shoulder_bicep_joint");
			output.position.push_back(-joint->position[i] - M_PI/2.);
			output.velocity.push_back(0.0);
			output.effort.push_back(0.0);
		}

		// convert pos if needed
	}

	joint_pub_.publish(output);
	
}

int main( int argc, char* argv[] )
{
	ros::init(argc,argv,"veltrobot_to_bandit");
	ros::NodeHandle nh;

	joint_pub_ = nh.advertise<sensor_msgs::JointState>("output_joint_state",10);
	ros::Subscriber joint_sub = nh.subscribe("input_joint_state",10,joint_cb);

	ros::spin();

	return 0;
}
