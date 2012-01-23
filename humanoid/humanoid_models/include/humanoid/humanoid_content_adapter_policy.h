/***************************************************************************
 *  include/humanoid/humanoid_content_adapter_policy.h
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
 *  * Neither the name of humanoid_models nor the names of its
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

#ifndef HUMANOIDMODELS_HUMANOID_HUMANOIDCONTENTADAPTER_H_
#define HUMANOIDMODELS_HUMANOID_HUMANOIDCONTENTADAPTER_H_

#include <quickdev/node_handle_policy.h>
#include <quickdev/callback_policy.h>

#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>

#include <humanoid/humanoid_features.h>

#include <std_msgs/String.h>

//template<class __RawStatesMsg>
//QUICKDEV_DECLARE_POLICY2( HumanoidContentAdapter, quickdev::NodeHandlePolicy, quickdev::MessageCallbackPolicy<__RawStatesMsg> )

QUICKDEV_DECLARE_POLICY( HumanoidContentAdapter, quickdev::NodeHandlePolicy )

template<class __RawStatesMsg>
//QUICKDEV_DECLARE_POLICY_CLASS2( HumanoidContentAdapter, __RawStatesMsg )
QUICKDEV_DECLARE_POLICY_CLASS( HumanoidContentAdapter )
{
    QUICKDEV_MAKE_POLICY_FUNCS( HumanoidContentAdapter )

public:
    typedef humanoid::_HumanoidStateArrayMsg _HumanoidStateArrayMsg;
    typedef quickdev::MessageCallbackPolicy<__RawStatesMsg> _RawStatesMessageCallbackPolicy;

    //QUICKDEV_DECLARE_POLICY_CONSTRUCTOR2( HumanoidContentAdapter, __RawStatesMsg )
    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( HumanoidContentAdapter )
    {
        //
    }

protected:
    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;

    void postInit()
    {
        QUICKDEV_GET_NODEHANDLE( nh_rel );

        multi_pub_.addPublishers<_HumanoidStateArrayMsg>( nh_rel, { "humanoid_states" } );
        multi_sub_.addSubscriber( nh_rel, "raw_states", &HumanoidContentAdapterPolicy::rawStatesCB, this );
    }

    QUICKDEV_ENABLE_INIT()
    {
        postInit();
        QUICKDEV_SET_INITIALIZED();
    }

    void update( const _HumanoidStateArrayMsg & humanoid_states )
    {
        // when using init(), it's useful to be able to detect whether our init() function as been called
        // one option is to simply read the member "initialized_"
        // alternatively, there are some simple macros for notifying/responding to situations where init() has not been called
        // these macros can be used anywhere within our policy as long as our init() function has been declared with QUICKDEV_ENABLE_INIT{}
        //
        // if we're not initialized yet, warn and continue
        // QUICKDEV_CHECK_INITIALIZED();
        //
        // if we're not initialized yet, warn and return immediately
        // QUICKDEV_ASSERT_INITIALIZED();
        //

        QUICKDEV_ASSERT_INITIALIZED();

        auto humanoid_states_ptr = quickdev::make_const_shared( humanoid_states );

        multi_pub_.publish( "humanoid_states", humanoid_states_ptr );
    }

    //QUICKDEV_DECLARE_MESSAGE_CALLBACK( rawStatesCB, __RawStatesMsg )
    //{
        //_RawStatesMessageCallbackPolicy::invokeCallback( msg );
    //}
};

#endif // HUMANOIDMODELS_HUMANOID_HUMANOIDCONTENTADAPTER_H_
