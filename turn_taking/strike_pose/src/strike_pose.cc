#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pose_model/pose_file.h>

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "strike_pose" );
	ros::NodeHandle nh;

	// load models from file
	std::string prefix;
	std::string model_filename;
	nh.param( "prefix", prefix, std::string("/home/dfseifer/diamondback-usc/stacks/usc-ros-pkg/trunk/turn_taking/pose_model") );
	nh.param( "model_filename", model_filename, std::string("poses.yaml") );

	ros::Publisher pose_pub = nh.advertise<sensor_msgs::JointState>( "target_pose", 1 );

	ros::Rate loop_rate(0.2);

	std::vector<pose_model::SimonPose> poses;
	poses = read_poses_from_yaml( model_filename, prefix );

	srand( time(NULL) );

	// until crashed
	while( ros::ok() )
	{
		// strike random pose
		int pose_idx = rand() % poses.size();

		sensor_msgs::JointState pose;
		pose.name = poses[pose_idx].joint_names;
		pose.position = poses[pose_idx].joint_poses;
		for( int i = 0; i < pose.name.size(); i++ )
		{
			pose.velocity.push_back(0.0);
			pose.effort.push_back(0.0);
		}

		pose_pub.publish(pose);

		// wait time
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
