#ifndef OCULESICS_OCULESICS_POLICY_H_
#define OCULESICS_OCULESICS_POLICY_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// specifies what all oculesics policies have in common
// these include:
// - a nodehandle for access to ROS
// - a function that directs the policy to select and look at a target
// - a function to step the policy's internal data models, if applicable
class OculesicsPolicy
{
public:
	enum State
	{
		IDLE,
		LOOK_AT_TARGET
	};
	
	typedef btVector3 _Target;
	
protected:
	// initialized on creation during class construction 
	ros::NodeHandle nh_rel_;
	State state_;
	ros::Time end_look_time_;
	
	// initialized after creation during class construction 
	ros::Rate loop_rate_;
	tf::TransformBroadcaster tf_broadcaster_;
	
public:
	OculesicsPolicy( ros::NodeHandle & nh, State state = IDLE, ros::Time end_look_time = ros::Time( 0 ) )
	 : nh_rel_( nh ), state_( state ), end_look_time_( end_look_time ), loop_rate_( 0 )
	{
		double loop_rate;
		nh_rel_.param( "loop_rate", loop_rate, 10.0 );
		loop_rate_ = ros::Rate( loop_rate );
	}
	
	// init( args... ) is called by the controller on construction
	// default init does nothing
	// overridden init functions can take any arguments
	void init(){};
	
	virtual double publishTargetAndGetLookTime() = 0;
	
	virtual void publishTarget( const _Target & target, std::string from_frame, std::string to_frame )
	{
		const static btQuaternion zero_quat( 0, 0, 0, 1 );
		tf_broadcaster_.sendTransform( tf::StampedTransform( tf::Transform( zero_quat, target ), ros::Time::now(), from_frame, to_frame ) );
	}
	
	inline ros::Time calculateEndLookTime( const double & look_duration ) const
	{
		return ros::Time::now() + ros::Duration( look_duration );
	}
	
	void step()
	{
		if( state_ == IDLE )
		{
			const double look_duration( publishTargetAndGetLookTime() );
			end_look_time_ = calculateEndLookTime( look_duration );
			printf( "Looking at target for %f seconds\n", look_duration );
			state_ = LOOK_AT_TARGET;
		}
		
		if( state_ == LOOK_AT_TARGET )
		{
			if( ros::Time::now() >= end_look_time_ ) state_ = IDLE;
		}
	}
};

#endif // OCULESICS_OCULESICS_POLICY_H_
