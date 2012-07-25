#include <map>
#include <ros/ros.h>
#include <openni_multitracker/UserStateArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>

std::vector<std::string> const FRAME_NAMES_
{
    "head"          ,
    "neck"          ,
    "torso"         ,
    "pelvis"        ,
    "right_shoulder",
    "right_elbow"   ,
    "right_hand"    ,
    "left_shoulder" ,
    "left_elbow"    ,
    "left_hand"     ,
    "right_hip"     ,
    "right_knee"    ,
    "right_foot"    ,
    "left_hip"      ,
    "left_knee"     ,
    "left_foot"
};

class PersonAutoSelector
{
public:

    std::map<std::string, std::string> people_;
    openni_multitracker::UserStateArray::_user_states_type user_states_cache_;
    ros::NodeHandle nh_rel_;
    int max_users_;
    ros::Rate * loop_rate_;
    std::string distance_frame_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Subscriber user_states_sub_;
    boost::mutex people_mutex_;

    PersonAutoSelector() : nh_rel_( "~" ), max_users_( 10 ), loop_rate_( NULL ), distance_frame_( "torso" )
    {
        // update transforms at this rate
        double loop_rate( 20 );
        nh_rel_.param( "loop_rate", loop_rate, loop_rate );
        loop_rate_ = new ros::Rate( loop_rate );
        nh_rel_.getParam( "max_users", max_users_ );
        nh_rel_.getParam( "distance_frame", distance_frame_ );

        user_states_sub_ = nh_rel_.subscribe( "user_states", 10, &PersonAutoSelector::userStatesCB, this );
    }

    // using the given TransformListener, look up a transform from
    // 'from_frame' to 'to_frame' and wait for at most 'wait_time' seconds
    // during lookup
    tf::StampedTransform
        lookupTransform(
            tf::TransformListener * listener,
            std::string from_frame,
            std::string to_frame,
            double wait_time = 0.1 )
    {
        tf::StampedTransform transform;

        if( !listener ) return transform;

        try
        {
    //      printf( "Waiting for transform..." );
            listener->waitForTransform(
                from_frame,
                to_frame,
                ros::Time( 0 ),
                ros::Duration( wait_time ) );
    //      printf( "Done.\n" );

            listener->lookupTransform(
                from_frame,
                to_frame,
                ros::Time(0),
                transform );
        }
        catch ( tf::TransformException & ex )
        {
            ROS_ERROR("While looking up %s -> %s : %s", from_frame.c_str(), to_frame.c_str(), ex.what() );
        }
        return transform;
    }

    void updatePeople()
    {
        people_mutex_.lock();

        ROS_DEBUG( "Processing %zu people...", people_.size() );
        typedef decltype( people_ ) _People;
        _People::mapped_type * closest_person = NULL;
        double shortest_vec_length;
        for ( auto person( people_.begin() ); person != people_.end(); ++person )
        {
            std::stringstream distance_target_frame_ss;
            distance_target_frame_ss << "/" << person->first << "/" << distance_frame_;
            const tf::StampedTransform
                observer_to_person( lookupTransform(
                    &tf_listener_,
                    "/openni_depth_tracking_frame",
                    distance_target_frame_ss.str() ) );

            if( !closest_person )
            {
                closest_person = &person->second;
                shortest_vec_length = observer_to_person.getOrigin().length();
            }
            else
            {
                const auto vec_length = observer_to_person.getOrigin().length();
                if( vec_length < shortest_vec_length )
                {
                    shortest_vec_length = vec_length;
                    closest_person = &person->second;
                }
            }
        }

        if( closest_person )
        {
            ROS_DEBUG( "Selecting closest person: %s", closest_person->c_str() );
            std::stringstream closest_person_name_ss;
            closest_person_name_ss << "/" << *closest_person << "/" << distance_frame_;
            for( size_t i = 0; i < FRAME_NAMES_.size(); ++i )
            {
                tf::Transform transform = lookupTransform( &tf_listener_, "/openni_depth_tracking_frame", "/" + *closest_person + "/" + FRAME_NAMES_[i] );
                tf_broadcaster_.sendTransform( tf::StampedTransform( transform, ros::Time::now(), "/openni_depth_tracking_frame", "/person/" + FRAME_NAMES_[i] ) );
            }
        }

        people_mutex_.unlock();
    }

    void userStatesCB( const openni_multitracker::UserStateArray::ConstPtr & user_states_msg )
    {
        user_states_cache_ = user_states_msg->user_states;

        if( !people_mutex_.try_lock() ) return;

        // grab all the most recent tracked users
        ROS_DEBUG( "Processing %zu users (limit %u)...", user_states_cache_.size(), (unsigned int)max_users_ );
        for( auto user = user_states_cache_.begin(); user != user_states_cache_.end(); ++user )
        {
            ROS_DEBUG( "%s %u", user->name.c_str(), user->is_tracked );
            if( user->is_tracked )
            {
                //tracked_users.push_back( *user );
                // if we haven't reached our capacity for tracking users and if we've found a new user
                if( people_.size() < (unsigned int)max_users_ && !people_.count( user->name ) )
                {
                    ROS_DEBUG( "Adding new user: %s", user->name.c_str() );
                    typedef decltype( people_ ) _MapType;
                    _MapType::mapped_type new_person( user->name );
                    people_.insert( _MapType::value_type( user->name, new_person ) );
                }
            }
            else
            {
                // if we know about an existing user and if we've lost that user
                if( people_.count( user->name ) )
                {
                    ROS_DEBUG( "Removing user: %s", user->name.c_str() );
                    people_.erase( user->name );
                }
            }
        }

        people_mutex_.unlock();
    }

    void spinOnce()
    {
        while( ros::ok() )
        {
            updatePeople();
            loop_rate_->sleep();
            ros::spinOnce();
        }
    }

    void spin()
    {
        spinOnce();
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "person_auto_selector" );
    PersonAutoSelector person_auto_selector;
    person_auto_selector.spin();
    return 0;
}
