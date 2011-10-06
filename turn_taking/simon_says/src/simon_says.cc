#include <ros/ros.h>

#include <pose_action/PoseAction.h>
#include <wait_for_response_action/WaitForResponseAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <pose_model/pose_file.h>
#include <simon_says/Prob.h>

bool spinning_ = true;

void spin()
{
	ros::Rate loop_rate(10);
	while( ros::ok() )
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	spinning_ = false;
	printf( "spin thread ending\n" );
}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "simon_says" );
	ros::NodeHandle nh;

	// load model files
  std::string prefix;
  std::string model_filename;

  nh.param( "prefix", prefix, std::string("/home/dfseifer/diamondback-usc/stacks/usc-ros-pkg/trunk/turn_taking/simon_says") );
  nh.param( "model_filename", model_filename, std::string("newer_poses.yaml") );

  ros::Publisher pose_pub = nh.advertise<sensor_msgs::JointState>( "target_pose", 1 );
  std::vector<pose_model::SimonPose> poses;
  poses = read_poses_from_yaml( model_filename, prefix );

  srand( time(NULL) );

	// spin up spin thread
	boost::thread spin_thread(spin);

	// find all necessary servers before starting game

  actionlib::SimpleActionClient<pose_action::PoseAction> pose_client("pose_action_server", true );
	actionlib::SimpleActionClient<wait_for_response_action::WaitForResponseAction> response_client("wait_for_response_server", true);

	response_client.waitForServer();
  pose_client.waitForServer();

	// game loop
	while( spinning_ )
	{
		// strike a pose
    int pose_idx = rand() % poses.size();

    sensor_msgs::JointState pose;
    pose.name = poses[pose_idx].joint_names;
    pose.position = poses[pose_idx].joint_poses;
    for( unsigned int i = 0; i < pose.name.size(); i++ )
    {
      pose.velocity.push_back(0.0);
      pose.effort.push_back(0.0);
    }

		pose_action::PoseGoal goal;
		goal.goal_state = pose;
		goal.pose_duration = ros::Duration(2.0);
		goal.move_duration = ros::Duration(0.5);

		pose_client.sendGoal(goal);
		ros::Duration(1.0).sleep();
		// wait for response
		wait_for_response_action::WaitForResponseGoal responseGoal;

		response_client.sendGoal(responseGoal);

		bool response_received = response_client.waitForResult(ros::Duration(15.0));

		if( ! response_received )
			ROS_INFO( "timeout" );
		else
		{
			wait_for_response_action::WaitForResponseResult responseResult = *(response_client.getResult());

			// feedback
			simon_says::Prob::Request  req;
			simon_says::Prob::Response res;

			req.goal_state = pose;
			req.current_state = responseResult.pose;

			ros::service::call("/pose_prob", req, res );

			ROS_INFO( "result: %f", res.prob );
		}

		usleep( 100*1000 );
	}

	printf( "main thread ending\n" );

	ros::spinOnce();

	return 0;
}
