/***************************************************************************
 *  include/quickdev/robot_controller_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_ROBOTCONTROLLERPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_ROBOTCONTROLLERPOLICY_H_

#include <quickdev/tf_manager_policy.h>
//#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

QUICKDEV_DECLARE_POLICY( RobotController, TfManagerPolicy )

template<class __MotorValsMsg>
QUICKDEV_DECLARE_POLICY_CLASS( RobotController )
{
    QUICKDEV_MAKE_POLICY_FUNCS( RobotController )

public:
    typedef TfManagerPolicy::_VelocityMsg _VelocityMsg;

protected:
    ros::MultiPublisher<> multi_pub_;
    //ros::MultiSubscriber<> multi_sub_;

    std::string
        robot_name_,
        world_frame_name_,
        robot_frame_name_,
        target_frame_name_,
        cmd_vel_topic_name_,
        motor_vals_topic_name_;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( RobotController ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

private:
    void postInit()
    {
        auto & nh_rel = NodeHandlePolicy::getNodeHandle();

        multi_pub_.addPublishers<__MotorValsMsg>( nh_rel, { motor_vals_topic_name_ } );
    }

public:
    QUICKDEV_ENABLE_INIT()
    {
        auto & nh_rel = NodeHandlePolicy::getNodeHandle();

        robot_name_ = policy::readPolicyParam<std::string>( nh_rel, "robot_name_param", "robot_name", "", args... );
        if( robot_name_.size() > 0 ) robot_name_.insert( 0, "/" );

        auto const world_frame_name_ = policy::readPolicyParam<std::string>( nh_rel, "world_frame_name_param", "world_frame_name", "/world", args... );

        robot_frame_name_ = robot_name_ + "/" + policy::readPolicyParam<std::string>( nh_rel, "robot_frame_name_param", "robot_frame_name", "base_link", args... );

        target_frame_name_ = robot_name_ + "/" + policy::readPolicyParam<std::string>( nh_rel, "target_frame_name_param", "target_frame_name", "desired_pose", args... );

        cmd_vel_topic_name_ = getMetaParamDef<std::string>( "cmd_vel_topic_name_param", robot_name_.size() > 0 ? robot_name_ + "/cmd_vel" : "cmd_vel", args... );
        motor_vals_topic_name_ = getMetaParamDef<std::string>( "motor_vals_topic_name_param", robot_name_.size() > 0 ? robot_name_ + "/motor_vals" : "motor_vals", args... );

        // make sure our frames are initialized in the manager (if any frame is already initialized it will not be modified here)
        nh_rel.setParam( "frame_pair0", world_frame_name_ + "," + target_frame_name_ );
        //TfManagerPolicy::registerFrames( world_frame_name_, target_frame_name_ );

        initPolicies<TfManagerPolicy>( "cmd_vel_topic_name_param", cmd_vel_topic_name_, args... );

        postInit();

        QUICKDEV_SET_INITIALIZED();
    }

    void update( const __MotorValsMsg & msg )
    {
        update( typename __MotorValsMsg::ConstPtr( new __MotorValsMsg( msg ) ) );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( update, typename __MotorValsMsg )
    {
        // publish all known transforms
        TfManagerPolicy::update();

        multi_pub_.publish( motor_vals_topic_name_, msg );
    }

    tf::StampedTransform getTransformToTarget( std::string const & target_frame_name ) const
    {
        QUICKDEV_CHECK_INITIALIZED();

        return lookupTransform( robot_frame_name_, target_frame_name, ros::Time::now() );
    }

    tf::StampedTransform getTransformToTarget() const
    {
        return lookupTransform( robot_frame_name_, target_frame_name_, ros::Time::now() );
    }
};

}

#endif // QUICKDEVCPP_QUICKDEV_ROBOTCONTROLLERPOLICY_H_
