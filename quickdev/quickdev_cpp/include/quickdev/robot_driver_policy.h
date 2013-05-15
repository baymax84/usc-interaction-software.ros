/***************************************************************************
 *  include/quickdev/robot_driver_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_ROBOTDRIVERPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_ROBOTDRIVERPOLICY_H_

#include <quickdev/node_handle_policy.h>
#include <quickdev/timed_policy.h>
#include <quickdev/callback_policy.h>
#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/threading.h>
#include <quickdev/serial.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

template<class __MotorValsMsg>
class RobotDriverPolicy : public GenericPolicyAdapter<NodeHandlePolicy, TimedPolicy<>, MessageCallbackPolicy<__MotorValsMsg> >
{
    QUICKDEV_MAKE_POLICY_FUNCS( RobotDriver )

protected:
    typedef MessageCallbackPolicy<__MotorValsMsg> _MessageCallbackPolicy;
    typedef GenericPolicyAdapter<NodeHandlePolicy, TimedPolicy<>, _MessageCallbackPolicy > _PolicyAdapter;
    typedef void _EmptyMsg;

    MessageCache<__MotorValsMsg> motor_vals_msg_cache_;

    ros::MultiSubscriber<> multi_sub_;
    // publisher for sensor data, etc
    ros::MultiPublisher<> multi_pub_;

    std::string
        robot_name_,
        motor_vals_topic_name_;

    template<class... __Args>
    RobotDriverPolicy( __Args&&... args ) : _PolicyAdapter( args... ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

private:
    void postInit()
    {
        auto & nh_rel = NodeHandlePolicy::getNodeHandle();

        multi_sub_.addSubscriber( nh_rel, motor_vals_topic_name_, &RobotDriverPolicy::motorValsCB, this );
    }

public:
    QUICKDEV_ENABLE_INIT()
    {
        auto & nh_rel = NodeHandlePolicy::getNodeHandle();

        const auto robot_name_param = getMetaParamDef<std::string>( "robot_name_param", "robot_name", args... );
        robot_name_ = ros::ParamReader<std::string, 1>::readParam( nh_rel, robot_name_param, "" );
        if( robot_name_.size() > 0 ) robot_name_.insert( 0, "/" );

        motor_vals_topic_name_ = getMetaParamDef<std::string>( "motor_vals_topic_name_param", robot_name_.size() > 0 ? robot_name_ + "/motor_vals" : "motor_vals" , args... );

        postInit();

        QUICKDEV_SET_INITIALIZED();
    }

private:
    QUICKDEV_DECLARE_MESSAGE_CALLBACK( motorValsCB, typename __MotorValsMsg )
    {
        TimedPolicy<>::update();

        QUICKDEV_TRY_UPDATE_CACHE( motor_vals_msg_cache_, msg );
        QUICKDEV_TRY_LOCK_OR_WARN( motor_vals_msg_cache_, "Dropping message [ %s ]", QUICKDEV_GET_MESSAGE_INST_NAME( msg ).c_str() );

        _MessageCallbackPolicy::invokeCallback( msg );
    }
};

}

#endif // QUICKDEVCPP_QUICKDEV_ROBOTDRIVERPOLICY_H_
