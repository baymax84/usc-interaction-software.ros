#include <ros/ros.h>

#include <pose_action/PoseAction.h>
#include <wait_for_response_action/WaitForResponseAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <pose_model/pose_file.h>
#include <sound_play/sound_play.h>

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
	ros::init( argc, argv, "build_model" );
	ros::NodeHandle nh;

	// load model files
  std::string prefix;
  std::string model_filename;
	bool add = false;

  nh.param( "prefix", prefix, std::string("/home/dfseifer/diamondback-usc/stacks/usc-ros-pkg/trunk/turn_taking/simon_says") );
  nh.param( "model_filename", model_filename, std::string("newer_poses.yaml") );
	nh.param( "add", add, false );
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

  sound_play::SoundClient sc(nh, "robotsound");

	int pose_idx = -1;
	// game loop

	FILE** ofiles = new FILE*[poses.size()];

	// open files
	for( int i = 0; i < poses.size(); i++ )
	{
		char name[256];
		name[0] = '\0';
		sprintf( name, "%d.pose", i );
		if( add )
		{
			ofiles[i] = fopen( name, "a" );
		}	
		else
		{
			ofiles[i] = fopen( name, "w" );
			for( int j = 0; j < 8; j++ )
			{
				fprintf( ofiles[i], "%0.4f ", poses[i].joint_poses[j] );
			}
			fprintf( ofiles[i], "\n" );
		}
	}

	while( spinning_ )
	{
		// strike a pose
    //pose_idx = rand() % poses.size();
		pose_idx = (pose_idx + 1) % poses.size();

		ROS_INFO( "posing: %d out of %d", pose_idx, (int) poses.size() );
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
		goal.move_duration = ros::Duration(0.5);
		goal.pose_duration = ros::Duration(2.0);

    // play initial prompt
    sc.playWave("/home/dfseifer/Downloads/snds/C/CAttention.wav");
		pose_client.sendGoal(goal);

		ros::Duration(1.0).sleep();

		// wait for response
		wait_for_response_action::WaitForResponseGoal responseGoal;
		response_client.sendGoal(responseGoal);

		bool response_received = response_client.waitForResult(ros::Duration(15.0));
    if( ! response_received )
    {
      // if failure
      ROS_INFO( "failed ... ");
      sc.playWave("/home/dfseifer/Downloads/snds/C/CKachingFailed.wav");

    }
    else
    {
      // if success
      ROS_INFO( "succeeded ... ");
      sc.playWave("/home/dfseifer/Downloads/snds/C/CBling.wav");

      // save to file
      const wait_for_response_action::WaitForResponseResult pose = *(response_client.getResult());
			/*
      time_t t = ::time(NULL);
      struct tm *tms = localtime(&t);
      char name[256];
      snprintf(name, sizeof(name), "response-%d-%d-%02d-%02d-%02d-%02d-%02d.txt", pose_idx,
                   tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
                   tms->tm_hour     , tms->tm_min  , tms->tm_sec);

      ROS_INFO( "opening file: [%s]", name );
      FILE* pose_file = fopen( name, "w" );
      if( !pose_file ) ROS_WARN( "problem opening file" );
			*/

      for( int i = 0; i < 8; i++ )
      {
        fprintf( ofiles[pose_idx], "%0.2f ", pose.pose.position[i] );
      }
      fprintf( ofiles[pose_idx], "\n" );
      //fclose( pose_file );
    }


		// feedback

		ros::Duration(5.0).sleep();
	}

	for( int i = 0; i < poses.size(); i++ )
	{
		fclose(ofiles[i]);
	}

	printf( "main thread ending\n" );

	ros::spinOnce();

	return 0;
}
