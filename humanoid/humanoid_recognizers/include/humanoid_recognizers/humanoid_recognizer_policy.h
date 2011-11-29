/***************************************************************************
 *  include/humanoid_recognizers/humanoid_recognizer_policy.h
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

#ifndef HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERPOLICY_H_
#define HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERPOLICY_H_

#include <quickdev/node_handle_policy.h>
#include <quickdev/callback_policy.h>
#include <quickdev/subscriber_policy.h>
#include <quickdev/publisher_policy.h>
#include <quickdev/threading.h>

#include <humanoid/humanoid.h>

#include <visualization_msgs/MarkerArray.h>

#include <map>

QUICKDEV_DECLARE_POLICY_NS( HumanoidRecognizer )
{
    typedef quickdev::NodeHandlePolicy _NodeHandlePolicy;
    typedef quickdev::MessageCallbackPolicy<humanoid::_HumanoidStateArrayMsg> _HumanoidStateArrayMessageCallbackPolicy;
    typedef quickdev::PublisherPolicy<> _PublisherPolicy;
    typedef quickdev::SubscriberPolicy<> _SubscriberPolicy;
}

QUICKDEV_DECLARE_POLICY( HumanoidRecognizer, _NodeHandlePolicy, _HumanoidStateArrayMessageCallbackPolicy, _PublisherPolicy, _SubscriberPolicy )
QUICKDEV_DECLARE_POLICY_CLASS( HumanoidRecognizer )
{
    QUICKDEV_MAKE_POLICY_FUNCS( HumanoidRecognizer )

public:
    typedef visualization_msgs::MarkerArray _MarkerArrayMsg;
    typedef visualization_msgs::Marker _MarkerMsg;
    typedef humanoid::_HumanoidStateArrayMsg _HumanoidStateArrayMsg;
    typedef humanoid::_HumanoidJointMsg _HumanoidJointMsg;
    typedef std::map<std::string, std::map<std::string, _HumanoidJointMsg> > _StatesMap;

protected:
    quickdev::MessageCache<_HumanoidStateArrayMsg> states_cache_;

    _StatesMap states_map_;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( HumanoidRecognizer ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    void postInit()
    {
        QUICKDEV_GET_NODEHANDLE( nh_rel );

        getMultiPub().addPublishers<_MarkerArrayMsg, _MarkerArrayMsg>( nh_rel, { "marker_array", "/visualization_marker_array" } );
        getMultiSub().addSubscriber( nh_rel, "humanoid_states", &HumanoidRecognizerPolicy::humanoidStatesCB, this );
    }

    QUICKDEV_ENABLE_INIT()
    {
        postInit();

        QUICKDEV_SET_INITIALIZED();
    }

    void update( _MarkerArrayMsg markers )
    {
        QUICKDEV_ASSERT_INITIALIZED();

        auto markers_ptr = quickdev::make_const_shared( markers );

        getMultiPub().publish( "marker_array", markers_ptr, "/visualization_marker_array", markers_ptr );
    }

    void buildStatesMap( const _HumanoidStateArrayMsg::ConstPtr & states_msg )
    {
        states_map_.clear();

        for( auto humanoid_msg = states_msg->states.begin(); humanoid_msg != states_msg->states.end(); ++humanoid_msg )
        {
            for( auto joint_msg = humanoid_msg->joints.begin(); joint_msg != humanoid_msg->joints.end(); ++joint_msg )
            {
                states_map_[humanoid_msg->name][joint_msg->name] = *joint_msg;
            }
        }
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( humanoidStatesCB, _HumanoidStateArrayMsg )
    {
        QUICKDEV_TRY_UPDATE_CACHE( states_cache_, msg );
        QUICKDEV_TRY_LOCK_OR_WARN( states_cache_, "Dropping message [ %s ]", QUICKDEV_GET_MESSAGE_INST_NAME( msg ).c_str() );

        QUICKDEV_GET_POLICY_NS( HumanoidRecognizer )::_HumanoidStateArrayMessageCallbackPolicy::invokeCallback( msg );
    }
};

#endif // HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERPOLICY_H_
