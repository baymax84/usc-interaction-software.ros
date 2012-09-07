#ifndef OCULESICS_PROBABILISTIC_OCULESIC_POLICY_H_
#define OCULESICS_PROBABILISTIC_OCULESIC_POLICY_H_

#include <oculesics/oculesic_policy.h>
#include <oculesics/random_generator.h>
#include <oculesics/param_array_reader.h>
#include <tf/transform_listener.h>

// specifies how to implement a probabilistic oculesic policy
class ProbabilisticOculesicPolicy : public OculesicPolicy
{
public:
    typedef OculesicPolicy _OculesicPolicy;

    RandomGenerator<DiscreteDistribution<>, 1, boost::mt19937> target_generator_;
    Range<double> look_duration_range_;
    std::vector<btVector3> points_;
    tf::TransformListener tf_listener_;

    std::string world_frame_name_;
    ParamArrayReader<std::string> frame_name_reader_;

    ProbabilisticOculesicPolicy( ros::NodeHandle & nh ) : _OculesicPolicy( nh )
    {
        nh_rel_.param( "look_time/min", look_duration_range_.min, 0.5 );
        nh_rel_.param( "look_time/max", look_duration_range_.max, 3.0 );

        // read in all params from "frame1_name" to "frame<n>_name"
        nh_rel_.param( "world_frame_name", world_frame_name_, std::string( "/world" ) );

        frame_name_reader_ = decltype( frame_name_reader_ )( nh_rel_, "frame", "_name", 1 );
        ParamArrayReader<double> frame_probability_reader( nh_rel_, "frame", "_probability", 1 );

        target_generator_.update( 0, frame_probability_reader.params_ );
    }

    void init()
    {
        printf( "Initialized!\n" );
    }

    double publishTargetAndGetLookTime()
    {
        printf( "publishTargetAndGetLookTime()\n" );
        auto target_index = target_generator_.sample()[0];

        tf::StampedTransform transform;
        try
        {
            ros::spinOnce();
            tf_listener_.waitForTransform( world_frame_name_, frame_name_reader_.params_[target_index], ros::Time(0), ros::Duration( 0.25 ) );
            tf_listener_.lookupTransform( world_frame_name_, frame_name_reader_.params_[target_index], ros::Time(0), transform );

            auto target = transform.getOrigin();

            publishTarget( _OculesicPolicy::_Target( target[0], target[1], target[2] ), world_frame_name_, "/pr2_head_look_target" );
        }
        catch ( tf::TransformException & ex )
        {
            ROS_ERROR("%s",ex.what() );
        }

        return ( look_duration_range_.max - look_duration_range_.min ) * target_generator_.getProbability( 0, target_index ) + look_duration_range_.min;
    }

    /*void publishTarget( const _OculesicPolicy::_Target & target )
    {
        printf( "publishTarget( %f %f %f )\n", target.x(), target.y(), target.z() );
    }*/
};

#endif // OCULESICS_PROBABILISTIC_OCULESIC_POLICY_H_
