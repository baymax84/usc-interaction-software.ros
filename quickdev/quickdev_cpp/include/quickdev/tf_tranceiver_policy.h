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

        std::string robot_frame_name_ = "seabee";
        std::string target_frame_name = "desired_frame";
    }

    void publishTransform( tf::StampedTransform const & transform, ros::Time const & frame_time )
    {
        auto new_transform( transform );
        new_transform.stamp_ = frame_time;
        publishTransform( new_transform );
    }

    void publishTransform( tf::StampedTransform const & transform, bool const & fix_timestamp = true )
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
            tf::StampedTransform(
                transform,
                frame_time,
                from_frame_id,
                to_frame_id ) );
    }

    tf::StampedTransform lookupTransform(
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

    tf::StampedTransform lookupTransform(
        _TfFrameId const & from_frame_id,
        ros::Time const & from_frame_time,
        _TfFrameId const & to_frame_id,
        ros::Time const & to_frame_time,
        _TfFrameId const & fixed_frame_id,
        ros::Duration const & wait_time,
        bool const & default_to_latest = true ) const
    {
        PRINT_DEBUG( "Looking up transform:\n [ %s-> %s ]\n( %f -> %f )...", from_frame_id.c_str(), to_frame_id.c_str(), from_frame_time.toSec(), to_frame_time.toSec() );
        tf::StampedTransform transform( btTransform( tf::createIdentityQuaternion() ), ros::Time::now(), from_frame_id, to_frame_id );

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

    tf::StampedTransform lookupTransform(
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

    tf::StampedTransform lookupTransform(
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

    tf::StampedTransform lookupTransform(
        _TfFrameId const & from_frame_id,
        _TfFrameId const & to_frame_id,
        ros::Time const & frame_time,
        ros::Duration const & wait_time,
        bool const & default_to_latest = true ) const
    {
        PRINT_DEBUG( "Looking up transform:\n [ %s-> %s ]\n( %f )...", from_frame_id.c_str(), to_frame_id.c_str(), frame_time.toSec() );
        tf::StampedTransform transform( btTransform( tf::createIdentityQuaternion() ), ros::Time::now(), from_frame_id, to_frame_id );

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

};

#undef DEFAULT_LOOKUP_TIME

}

#endif // QUICKDEVCPP_QUICKDEV_TFTRANCEIVERPOLICY_H_
