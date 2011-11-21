#include <iostream>
#include <ros/shared_memory_publisher.h>
#include <ros/shared_memory_subscriber.h>
#include <test_matlab/Vector3.h>

int main( int argc, char ** argv )
{
	if( argc >= 2 )
	{
		//usleep( 1000*1000 );
		ros::SharedMemorySubscriber sub( argv[1] );
		ros::SharedMemoryPublisher pub( sub.getStorage().getName() );

		auto msg = sub.fetch<test_matlab::Vector3>();
		std::cout << msg.x << ", " << msg.y << ", " << msg.z << std::endl;

		msg.x *= 2;
		msg.y *= 2;
		msg.z *= 2;

		pub.publish( msg );
	}

	return 0;
}
