#include <oculesics/oculesic_controller.h>
#include <oculesics/probabilistic_oculesic_policy.h>

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "probabilistic_oculesic_controller" );
    ros::NodeHandle nh( "~" );

    OculesicController<ProbabilisticOculesicPolicy> probabilistic_oculesic_controller( nh );
    probabilistic_oculesic_controller.spin();

    return 0;
};
