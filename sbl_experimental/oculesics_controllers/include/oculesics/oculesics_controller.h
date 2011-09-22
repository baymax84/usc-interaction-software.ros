#ifndef OCULESICS_OCULESICS_CONTROLLER_H_
#define OCULESICS_OCULESICS_CONTROLLER_H_

#include <ros/ros.h>
//#include <vector>
//#include <stdio.h>

// the generic oculesics controller
// this class is designed to function as an adapter between the specified policy and any generic ros-related content
// the oculesics controller publishes a tf frame that represents the point at which the robot should look
// there will be a separate gaze controller that can be used to convert the tf frames into joint angles for a robot
template<class __Policy>
class OculesicsController : public __Policy
{
public:
	template<class... __Args>
	OculesicsController( ros::NodeHandle & nh, __Args... args ) : __Policy( nh )
	{
		__Policy::init( args... );
	}
	
	void spinOnce()
	{
		__Policy::step();
	}
	
	void spin()
	{
		while( ros::ok() )
		{
			spinOnce();
			ros::spinOnce();
			this->loop_rate_.sleep();
		}
	}
};

#endif // OCULESICS_OCULESICS_CONTROLLER_H_
