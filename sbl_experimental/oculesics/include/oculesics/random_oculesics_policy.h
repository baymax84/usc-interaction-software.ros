#ifndef OCULESICS_RANDOM_OCULESICS_POLICY_H_
#define OCULESICS_RANDOM_OCULESICS_POLICY_H_

#include <oculesics/oculesics_policy.h>
#include <oculesics/random_generator.h>

// specifies how to implement a random oculesics policy
class RandomOculesicsPolicy : public OculesicsPolicy
{
public:
	typedef OculesicsPolicy _OculesicsPolicy;
	
	RandomGenerator<boost::uniform_real<>, 3, boost::mt19937> target_generator_;
	RandomGenerator<boost::uniform_real<>, 1, boost::mt19937> duration_generator_;
	
	std::string world_frame_name_;
	
	RandomOculesicsPolicy( ros::NodeHandle & nh ) : _OculesicsPolicy( nh )
	{
		nh_rel_.param( "world_frame_name", world_frame_name_, std::string( "/world" ) );
		Range<double> range;
		nh_rel_.param( "look_time/min", range.min, 0.5 );
		nh_rel_.param( "look_time/max", range.max, 3.0 );
		
		duration_generator_.update( 0, range.min, range.max );
		
		nh_rel_.param( "target_bounds/x/min", range.min, 0.0 );
		nh_rel_.param( "target_bounds/x/max", range.max, 2.0 );
		
		target_generator_.update( 0, range.min, range.max );
		
		nh_rel_.param( "target_bounds/y/min", range.min, -1.0 );
		nh_rel_.param( "target_bounds/y/max", range.max, 1.0 );
		
		target_generator_.update( 1, range.min, range.max );
		
		nh_rel_.param( "target_bounds/z/min", range.min, -1.0 );
		nh_rel_.param( "target_bounds/z/max", range.max, 1.0 );
		
		target_generator_.update( 2, range.min, range.max );
	}
	
	void init()
	{
		printf( "Initialized!\n" );
	}
	
	double publishTargetAndGetLookTime()
	{
		printf( "publishTargetAndGetLookTime()\n" );
		auto target = target_generator_.sample();
		publishTarget( _OculesicsPolicy::_Target( target[0], target[1], target[2] ), world_frame_name_, "/target" );
		return duration_generator_.sample()[0];
	}
	
	/*void publishTarget( const _OculesicsPolicy::_Target & target )
	{
		printf( "publishTarget( %f %f %f )\n", target.x(), target.y(), target.z() );
	}*/
};

#endif // OCULESICS_RANDOM_OCULESICS_POLICY_H_
