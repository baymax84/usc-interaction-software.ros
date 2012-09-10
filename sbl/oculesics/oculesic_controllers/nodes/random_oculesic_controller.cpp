#include <oculesics/oculesic_controller.h>
#include <oculesics/random_oculesic_policy.h>

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "random_oculesic_controller" );
    ros::NodeHandle nh( "~" );

    OculesicController<RandomOculesicPolicy> random_oculesic_controller( nh );
    random_oculesic_controller.spin();

    return 0;
};
