/***************************************************************************
 *  include/quickdev/timed_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_TIMEDPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_TIMEDPOLICY_H_

#include <quickdev/policy.h>
#include <ros/time.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

QUICKDEV_DECLARE_POLICY( Timed, Policy )

template<unsigned int __Id__ = 0>
QUICKDEV_DECLARE_POLICY_CLASS( Timed )
{
    QUICKDEV_MAKE_POLICY_FUNCS( Timed )

public:
    typedef ros::Time _Time;
    typedef double _Duration;

protected:
    _Time last_time_;
    _Time now_;
    _Duration dt_;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( Timed ),
        last_time_( 0 ),
        now_( 0 ),
        dt_( 0 )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    QUICKDEV_ENABLE_UPDATE()
    {
        if( now_.toSec() == 0 )
        {
            now_ = ros::Time::now();
            last_time_ = now_;
            return;
        }

        last_time_ = now_;
        now_ = ros::Time::now();
        dt_ = ( now_ - last_time_ ).toSec();
    }

    inline const _Time & now(){ return now_; }
    inline const _Time & last(){ return last_time_; }
    inline const _Duration & dt(){ return dt_; }
};

}

#endif // QUICKDEVCPP_QUICKDEV_TIMEDPOLICY_H_
