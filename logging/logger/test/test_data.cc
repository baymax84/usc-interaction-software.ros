#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main( int argc, char* argv[] )
{
	ros::init(argc,argv,"test_data");
	ros::NodeHandle nh;
	
	ros::Publisher int_pub = nh.advertise<std_msgs::Int32>("test_data",100);
	ros::Rate loop_rate(10);

	int i = 0;
	std_msgs::Int32 int_msg;

	while( ros::ok() )
	{
		int_msg.data = i++;
		int_pub.publish(int_msg);

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
