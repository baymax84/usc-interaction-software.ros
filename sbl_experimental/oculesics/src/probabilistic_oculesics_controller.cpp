#include <oculesics/oculesics_controller.h>
#include <oculesics/probabilistic_oculesics_policy.h>

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "probabilistic_oculesics_controller" );
	ros::NodeHandle nh( "~" );
	
	OculesicsController<ProbabilisticOculesicsPolicy> probabilistic_oculesics_controller( nh );
	probabilistic_oculesics_controller.spin();
	
	return 0;
};
