#include <oculesics/oculesics_controller.h>
#include <oculesics/random_oculesics_policy.h>

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "random_oculesics_controller" );
	ros::NodeHandle nh( "~" );
	
	OculesicsController<RandomOculesicsPolicy> random_oculesics_controller( nh );
	random_oculesics_controller.spin();
	
	return 0;
};
