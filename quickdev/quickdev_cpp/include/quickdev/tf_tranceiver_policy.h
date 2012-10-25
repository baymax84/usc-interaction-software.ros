/***************************************************************************
 *  include/quickdev/tf_tranceiver_policy.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of usc-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#ifndef QUICKDEVCPP_QUICKDEV_TFTRANCEIVERPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_TFTRANCEIVERPOLICY_H_

#include <quickdev/policy.h>
#include <quickdev/threading.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

#define DEFAULT_LOOKUP_TIME ros::Time( 0 )

QUICKDEV_DECLARE_POLICY( TfTranceiver, Policy )

QUICKDEV_DECLARE_POLICY_CLASS( TfTranceiver )
{
    QUICKDEV_MAKE_POLICY_FUNCS( TfTranceiver )

protected:
    tf::TransformBroadcaster tf_publisher_;
    tf::TransformListener tf_listener_;

public:
    typedef std::string _TfFrameId;
    typedef tf::StampedTransform _StampedTransform;
    typedef btTransform _Transform;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( TfTranceiver )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    void publishTransform( _StampedTransform const & transform, ros::Time const & frame_time )
    {
        auto new_transform( transform );
        new_transform.stamp_ = frame_time;
        publishTransform( new_transform );
    }

    void publishTransform( _StampedTransform const & transform, bool const & fix_timestamp = true )
    {
        if( transform.frame_id_.size() == 0 || transform.child_frame_id_.size() == 0 )
        {
            PRINT_WARN( "Cannot publish StampedTransform with empty source frame or target frame id:\n[ %s -> %s ] : %f", transform.frame_id_.c_str(), transform.child_frame_id_.c_str(), transform.stamp_.toSec() );
            return;
        }

        if( transform.stamp_.is_zero() )
        {
            if( fix_timestamp ) return publishTransform( transform, ros::Time::now() );
            else
            {
                PRINT_WARN( "Cannot publish StampedTransform with zero timestamp" );
                return;
            }
        }
        PRINT_DEBUG( "Publishing %s -> %s [%f]", transform.frame_id_.c_str(), transform.child_frame_id_.c_str(), transform.stamp_.toSec() );
        tf_publisher_.sendTransform( transform );
    }

    void publishTransform(
        tf::Transform const & transform,
        _TfFrameId const & from_frame_id,
        _TfFrameId const & to_frame_id )
    {
        publishTransform(
            transform,
            from_frame_id,
            to_frame_id,
            ros::Time::now() );
    }

    void publishTransform(
        tf::Transform const & transform,
        _TfFrameId const & from_frame_id,
        _TfFrameId const & to_frame_id,
        ros::Time const & frame_time )
    {
        publishTransform(
            _StampedTransform(
                transform,
                frame_time,
                from_frame_id,
                to_frame_id ) );
    }

    _StampedTransform lookupTransform(
        _TfFrameId const & from_frame_id,
        ros::Time const & from_frame_time,
        _TfFrameId const & to_frame_id,
        ros::Time const & to_frame_time,
        _TfFrameId const & fixed_frame_id,
        double const & wait_time = 0.05,
        bool const & default_to_latest = true ) const
    {
        return lookupTransform(
            from_frame_id,
            from_frame_time,
            to_frame_id,
            to_frame_time,
            fixed_frame_id,
            ros::Duration( wait_time ),
            default_to_latest );
    }

    _StampedTransform lookupTransform(
        _TfFrameId const & from_frame_id,
        ros::Time const & from_frame_time,
        _TfFrameId const & to_frame_id,
        ros::Time const & to_frame_time,
        _TfFrameId const & fixed_frame_id,
        ros::Duration const & wait_time,
        bool const & default_to_latest = true ) const
    {
        PRINT_DEBUG( "Looking up transform:\n [ %s-> %s ]\n( %f -> %f )...", from_frame_id.c_str(), to_frame_id.c_str(), from_frame_time.toSec(), to_frame_time.toSec() );
        _StampedTransform transform( btTransform( btQuaternion( 0, 0, 0, 1 ) ), ros::Time::now(), from_frame_id, to_frame_id );

        if( transformExists(
            from_frame_id,
            from_frame_time,
            to_frame_id,
            to_frame_time,
            fixed_frame_id ) )
        {
            tf_listener_.waitForTransform(
                from_frame_id,
                from_frame_time,
                to_frame_id,
                to_frame_time,
                fixed_frame_id,
                wait_time );

            tf_listener_.lookupTransform(
                from_frame_id,
                from_frame_time,
                to_frame_id,
                to_frame_time,
                fixed_frame_id,
                transform );

            PRINT_DEBUG( "OK\n" );
        }
        else
        {
            PRINT_DEBUG(
                "Cannot find transform from %s to %s via %s at the given times",
                from_frame_id.c_str(),
                to_frame_id.c_str(),
                fixed_frame_id.c_str() );

            if( default_to_latest )
            {
                PRINT_DEBUG( "Attempting to look up latest transform..." );
                return lookupTransform( from_frame_id, to_frame_id, ros::Time( 0 ), wait_time, false );
            }
            else
            {
                std::string const error_str = "Lookup of " + from_frame_id + " -> " + to_frame_id + " via " + fixed_frame_id + " failed.";
                throw tf::TransformException( error_str );
            }
        }
        transform.setRotation( transform.getRotation().normalized() );
        return transform;
    }

    _StampedTransform lookupTransform(
        _TfFrameId const & from_frame_id,
        _TfFrameId const & to_frame_id,
        double const & wait_time = 0.05,
        bool const & default_to_latest = true ) const
    {
        return lookupTransform(
            from_frame_id,
            to_frame_id,
            DEFAULT_LOOKUP_TIME,
            wait_time,
            default_to_latest );
    }

    _StampedTransform lookupTransform(
        _TfFrameId const & from_frame_id,
        _TfFrameId const & to_frame_id,
        ros::Time const & frame_time,
        double const & wait_time = 0.05,
        bool const & default_to_latest = true ) const
    {
        return lookupTransform(
            from_frame_id,
            to_frame_id,
            frame_time,
            ros::Duration( wait_time ),
            default_to_latest );
    }

    _StampedTransform lookupTransform(
        _TfFrameId const & from_frame_id,
        _TfFrameId const & to_frame_id,
        ros::Time const & frame_time,
        ros::Duration const & wait_time,
        bool const & default_to_latest = true ) const
    {
        PRINT_DEBUG( "Looking up transform:\n [ %s-> %s ]\n( %f )...", from_frame_id.c_str(), to_frame_id.c_str(), frame_time.toSec() );
        _StampedTransform transform( btTransform( btQuaternion( 0, 0, 0, 1 ) ), ros::Time::now(), from_frame_id, to_frame_id );

        if( transformExists(
            from_frame_id,
            to_frame_id,
            frame_time ) )
        {
            tf_listener_.waitForTransform(
                from_frame_id,
                to_frame_id,
                frame_time,
                wait_time );

            tf_listener_.lookupTransform(
                from_frame_id,
                to_frame_id,
                frame_time,
                transform );

            PRINT_DEBUG( "OK" );
        }
        else
        {
            PRINT_DEBUG(
                "Cannot find transform from %s to %s at the given time",
                from_frame_id.c_str(),
                to_frame_id.c_str() );

            if( default_to_latest )
            {
                PRINT_DEBUG( "Attempting to look up latest transform..." );
                return lookupTransform( from_frame_id, to_frame_id, ros::Time( 0 ), wait_time, false );
            }
            else
            {
                std::string const error_str = "Lookup of " + from_frame_id + " -> " + to_frame_id + " failed.";
                throw tf::TransformException( error_str );
            }
        }

        transform.setRotation( transform.getRotation().normalized() );
        return transform;
    }

    bool transformExists(
        _TfFrameId const & from_frame_id,
        _TfFrameId const & to_frame_id ) const
    {
        return transformExists(
            from_frame_id,
            to_frame_id,
            DEFAULT_LOOKUP_TIME );
    }

    bool transformExists(
        _TfFrameId const & from_frame_id,
        _TfFrameId const & to_frame_id,
        ros::Time const & frame_time ) const
    {
        // to_frame and from_frame are flipped in the tf api here
        return tf_listener_.canTransform(
            to_frame_id,
            from_frame_id,
            frame_time );
    }

    bool transformExists(
        _TfFrameId const & from_frame_id,
        ros::Time const & from_frame_time,
        _TfFrameId const & to_frame_id,
        ros::Time const & to_frame_time,
        _TfFrameId const & fixed_frame_id ) const
    {
        // to_frame and from_frame are flipped in the tf api here
        return tf_listener_.canTransform(
            to_frame_id,
            to_frame_time,
            from_frame_id,
            from_frame_time,
            fixed_frame_id );
    }

    template<class... __Args>
    _StampedTransform tryLookupTransform( __Args&&... args )
    {
        auto const & from_frame_id = QUICKDEV_GET_ARG( 0, __Args, args );
        auto const & to_frame_id = QUICKDEV_GET_ARG( 1, __Args, args );

        _StampedTransform transform( btTransform( tf::createIdentityQuaternion() ), ros::Time::now(), from_frame_id, to_frame_id );

        try
        {
            transform = lookupTransform( std::forward<__Args>( args )... );
        }
        catch( std::exception const & e )
        {
            PRINT_ERROR( "%s", e.what() );
        }
        return transform;
    }

    template<class... __Args>
    _StampedTransform waitForAndLookupTransform( __Args&&... args )
    {
        auto const & from_frame_id = QUICKDEV_GET_ARG( 0, __Args, args );
        auto const & to_frame_id = QUICKDEV_GET_ARG( 1, __Args, args );
        auto const check_rate = QUICKDEV_GET_ARG( 2, __Args, args );
        auto const max_attempts = QUICKDEV_GET_ARG( 3, __Args, args );

        _StampedTransform transform( btTransform( tf::createIdentityQuaternion() ), ros::Time::now(), from_frame_id, to_frame_id );

        if( waitForTransform( from_frame_id, to_frame_id, check_rate, max_attempts ) )
        {
            try
            {
                transform = lookupTransform( std::forward<__Args>( args )... );
            }
            catch( std::exception const & e )
            {
                PRINT_ERROR( "%s", e.what() );
            }
        }
        return transform;
    }

    bool waitForTransform( _TfFrameId const & from_frame_id, _TfFrameId const & to_frame_id, double const & check_rate_hz = 10, long unsigned int const & max_attempts = 0 )
    {
        PRINT_DEBUG( "Checking for transform %s -> %s", from_frame_id.c_str(), to_frame_id.c_str() );

        long unsigned int num_attempts = 0;
        ros::Rate check_rate( check_rate_hz );

        ros::spinOnce();

        while( ros::ok() && ( max_attempts == 0 || num_attempts < max_attempts ) && !transformExists( from_frame_id, to_frame_id ) )
        {
            std::string attempts_info;
            if( max_attempts != 0 )
            {
                num_attempts ++;
                std::stringstream attempts_info_ss;
                attempts_info_ss << "( attempt " << num_attempts << " / " << max_attempts << " )";
                attempts_info = attempts_info_ss.str();
            }

            PRINT_WARN( "Waiting for transform %s -> %s%s...", from_frame_id.c_str(), to_frame_id.c_str(), attempts_info.c_str() );

            check_rate.sleep();
            ros::spinOnce();
        }

        return ros::ok() && ( max_attempts == 0 || num_attempts < max_attempts );
    }

};

#undef DEFAULT_LOOKUP_TIME

}

#endif // QUICKDEVCPP_QUICKDEV_TFTRANCEIVERPOLICY_H_
