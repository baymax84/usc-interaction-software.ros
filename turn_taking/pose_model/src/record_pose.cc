#include <ros/ros.h>
#include <sound_play/sound_play.h>

#include <wait_for_response_action/WaitForResponseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "record_pose" );
	ros::NodeHandle nh;
	sound_play::SoundClient sc(nh, "robotsound");

	// get action servers
  actionlib::SimpleActionClient<wait_for_response_action::WaitForResponseAction> response_client("wait_for_response_server", true);

	response_client.waitForServer();

	ros::Rate loop_rate(0.1);

	ros::AsyncSpinner spin(2);
	spin.start();
	while( ros::ok() )
	{
		ROS_INFO( "shooting ... " );

		// play initial prompt
		sc.playWave("/home/dfseifer/Downloads/snds/C/CAttention.wav");

		wait_for_response_action::WaitForResponseGoal responseGoal;
		response_client.sendGoal(responseGoal);

		bool response_received = response_client.waitForResult(ros::Duration(10.0));

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
	    time_t t = ::time(NULL);
	    struct tm *tms = localtime(&t);
  	  char name[256];
    	snprintf(name, sizeof(name), "pose-%d-%02d-%02d-%02d-%02d-%02d.txt",
                   tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
                   tms->tm_hour     , tms->tm_min  , tms->tm_sec);

			ROS_INFO( "opening file: [%s]", name );
			FILE* pose_file = fopen( name, "w" );
			if( !pose_file ) ROS_WARN( "problem opening file" );
			for( int i = 0; i < 8; i++ )
			{
				fprintf( pose_file, "%0.2f ", pose.pose.position[i] );
			}
			fprintf( pose_file, "\n" );
			fclose( pose_file );
		}

		ros::spinOnce();
		ros::Duration(2.0).sleep();
	}

	return 0;
}
