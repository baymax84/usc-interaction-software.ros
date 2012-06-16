/***************************************************************************
 *  include/quickdev/runable_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_RUNABLEPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_RUNABLEPOLICY_H_

#include <quickdev/node_handle_policy.h>
#include <ros/rate.h>
#include <boost/thread.hpp>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

QUICKDEV_DECLARE_POLICY( Runable, NodeHandlePolicy )

QUICKDEV_DECLARE_POLICY_CLASS( Runable )
{
    QUICKDEV_MAKE_POLICY_FUNCS( Runable )

private:
    double loop_rate_hz_;
    double loop_rate_seconds_;
    bool spin_ros_thread_;
    double ros_loop_rate_hz_;

    boost::shared_ptr<ros::Rate> loop_rate_;
    boost::shared_ptr<ros::Rate> ros_loop_rate_;
    boost::shared_ptr<boost::thread> ros_thread_ptr_;
    bool run_;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( Runable ),
        spin_ros_thread_( false ),
        run_( false ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );

        preInit();

        printPolicyActionDone( "create", this );
    }

    void preInit()
    {
        auto & nh_rel = NodeHandlePolicy::getNodeHandle();

        loop_rate_hz_ = quickdev::ParamReader::readParam<decltype( loop_rate_hz_ )>( nh_rel, "loop_rate", 10 );
        loop_rate_seconds_ = 1.0 / loop_rate_hz_;
        loop_rate_ = boost::make_shared<ros::Rate>( loop_rate_hz_ );
    }

    QUICKDEV_ENABLE_INIT()
    {
        auto & nh_rel = NodeHandlePolicy::getNodeHandle();

        spin_ros_thread_ = policy::readPolicyParam<decltype( spin_ros_thread_ )>( nh_rel, "spin_ros_thread_param", "spin_ros_thread", false, args... );
        if( spin_ros_thread_ )
        {
            ros_loop_rate_hz_ = quickdev::ParamReader::readParam<decltype( ros_loop_rate_hz_ )>( nh_rel, "ros_loop_rate", 10 );
            ros_loop_rate_ = boost::make_shared<ros::Rate>( ros_loop_rate_hz_ );
        }

        QUICKDEV_SET_INITIALIZED();
    }

    void updateRos()
    {
        while( run_ && ros::ok() )
        {
            ros::spinOnce();
            if( ros_loop_rate_ ) ros_loop_rate_->sleep();
        }
    }

    QUICKDEV_DECLARE_ACCESSOR2( loop_rate_, LoopRate )
    QUICKDEV_DECLARE_CONST_ACCESSOR( loop_rate_hz_, LoopRateHz )
    QUICKDEV_DECLARE_CONST_ACCESSOR( loop_rate_seconds_, LoopRateSeconds )
    QUICKDEV_DECLARE_CONST_ACCESSOR( spin_ros_thread_, SpinNewThread )
    QUICKDEV_DECLARE_CONST_ACCESSOR( ros_loop_rate_hz_, ROSLoopRateHz )

    virtual void setup(){}

    virtual void spinFirst(){}

    virtual void spinOnce(){}

    virtual void spin()
    {
        run_ = true;

        setup();

        if( spin_ros_thread_ )
        {
            PRINT_INFO( "--------------------" );
            PRINT_INFO( ">>>>> Spinning dedicated ROS thread at %f Hz...", ros_loop_rate_hz_ );
            ros_thread_ptr_ = boost::make_shared<boost::thread>( &RunablePolicy::updateRos, this );
        }

        PRINT_INFO( "--------------------" );
        PRINT_INFO( ">>>>> Starting pre-spin..." );

        spinFirst();

        PRINT_INFO( "--------------------" );
        PRINT_INFO( "<<<<< Done pre-spin" );

        PRINT_INFO( "--------------------" );
        PRINT_INFO( ">>>>> Spinning at %f Hz...", loop_rate_hz_ );

        while( run_ && ros::ok() )
        {
            spinOnce();
            // regardless of whether the user is blocking in ::spinOnce, we want to update ROS callbacks if we get here
            // if the user has enabled asynchronous ROS updates, their callbacks may be called both asynchronously and at this point
            ros::spinOnce();
            if( loop_rate_ ) loop_rate_->sleep();
        }

        PRINT_INFO( "<<<<< Main loop finished" );
    }

    virtual void interrupt()
    {
        PRINT_INFO( ">>>>> Interrupting main loop..." );
        run_ = false;
    }
};

}

#endif // QUICKDEVCPP_QUICKDEV_RUNABLEPOLICY_H_
