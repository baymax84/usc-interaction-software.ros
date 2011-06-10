#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


bool is_moving = true;
FILE* pose_file = NULL;
double threshold_ = 0.0;

void joint_cb( const sensor_msgs::JointStateConstPtr &vel )
{
	bool moving = false;

  for( int i = 0; i < vel->velocity.size(); i++ )
  {
    if( vel->velocity[i] > threshold_ )
    {
      moving  = true;
      break;
    }
  }
  ROS_INFO( "moving: %d", moving );

	if( is_moving && !moving )
	{
		// start file
    time_t t = ::time(NULL);
    struct tm *tms = localtime(&t);
    char name[256];
    snprintf(name, sizeof(name), "%d-%02d-%02d-%02d-%02d-%02d-pose.txt",
                   tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
                   tms->tm_hour     , tms->tm_min  , tms->tm_sec);

    ROS_INFO( "opening file: [%s]", name );
    pose_file = fopen( name, "w" );
		if( !pose_file ) ROS_WARN( "problem openning file" );
	}

	if( !moving )
	{
		// write line to file
		if( pose_file )
		{
			fprintf( pose_file, "%0.2f ", vel->header.stamp.toSec() );
			for( int i = 0; i < vel->position.size(); i++ )
			{
				fprintf( pose_file, "%0.2f ", vel->position[i] );
			}
			fprintf( pose_file, "\n" );
		}
	}

	if( !is_moving && moving )
	{
		// finish writing to file
		fclose( pose_file );
		pose_file = NULL;
	}


	is_moving = moving;
}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "pose_writer" );
	ros::NodeHandle nh;
	ros::Rate loop_rate( 10 );

	nh.param( "threshold", threshold_, 0.03 );
	ros::Subscriber joint_sub = nh.subscribe( "joints_with_vel", 1, joint_cb );

	while( ros::ok() )
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
