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

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

QUICKDEV_DECLARE_POLICY( Runable, NodeHandlePolicy )

QUICKDEV_DECLARE_POLICY_CLASS( Runable )
{
    QUICKDEV_MAKE_POLICY_FUNCS( Runable )

private:
    double loop_rate_hz_;
    double loop_rate_seconds_;

    boost::shared_ptr<ros::Rate> loop_rate_;
    bool run_;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( Runable ),
        run_( false )
    {
        printPolicyActionStart( "create", this );

        preInit();

        printPolicyActionDone( "create", this );
    }

    void preInit()
    {
        auto & nh_rel = NodeHandlePolicy::getNodeHandle();

        loop_rate_hz_ = ros::ParamReader<double, 1>::readParam( nh_rel, "loop_rate", 10 );
        loop_rate_seconds_ = 1.0 / loop_rate_hz_;
        loop_rate_ = make_shared( new ros::Rate( loop_rate_hz_ ) );
    }

    QUICKDEV_DECLARE_CONST_ACCESSOR( loop_rate_hz_, LoopRateHz );
    QUICKDEV_DECLARE_CONST_ACCESSOR( loop_rate_seconds_, LoopRateSeconds );

    virtual void spinFirst(){}

    virtual void spinOnce(){}

    virtual void spin()
    {
        run_ = true;

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
