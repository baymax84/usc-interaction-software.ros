/***************************************************************************
 *  include/bandit_calibration/calibration_node.h
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
 *  * Neither the name of interaction-ros-pkg nor the names of its
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

#ifndef BANDITCALIBRATIONPROTOTYPE_BANDITCALIBRATION_CALIBRATIONNODE_H_
#define BANDITCALIBRATIONPROTOTYPE_BANDITCALIBRATION_CALIBRATIONNODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/reconfigure_policy.h>

// objects
#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>

// msgs
#include <sensor_msgs/JointState.h>

// reconfigure
#include <bandit_calibration_prototype/CalibrationConfig.h>

// other
#include <quickdev/numeric_unit_conversions.h>

typedef sensor_msgs::JointState _JointStateMsg;

typedef bandit_calibration_prototype::CalibrationConfig _CalibrationConfig;

typedef quickdev::ReconfigurePolicy<_CalibrationConfig> _CalibrationConfigPolicy;

QUICKDEV_DECLARE_NODE( Calibration, _CalibrationConfigPolicy )

QUICKDEV_DECLARE_NODE_CLASS( Calibration )
{
    ros::MultiSubscriber<> multi_sub_;
    ros::MultiPublisher<> multi_pub_;

    _JointStateMsg last_joint_state_msg_;

    XmlRpc::XmlRpcValue joints_config_values_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( Calibration )
    {

    }

    QUICKDEV_SPIN_FIRST()
    {
        _CalibrationConfigPolicy::registerCallback( quickdev::auto_bind( &CalibrationNode::reconfigureCB, this ) );

        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_sub_.addSubscriber( nh_rel, "joint_states", &CalibrationNode::jointStatesCB, this );
        multi_pub_.addPublishers<_JointStateMsg>( nh_rel, { "joint_state_command" } );

        joints_config_values_ = ros::ParamReader<XmlRpc::XmlRpcValue, 1>::readParam( nh_rel, "joints" );

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( jointStatesCB, _JointStateMsg )
    {
        last_joint_state_msg_ = *msg;
    }

    QUICKDEV_DECLARE_RECONFIGURE_CALLBACK( reconfigureCB, _CalibrationConfig )
    {
        _JointStateMsg joint_state_msg;
        joint_state_msg.name = { "eyebrows_joint", "mouth_top_joint", "mouth_bottom_joint" };
        joint_state_msg.position =
        {
            Radian( Degree( config.eyebrows_joint_pos * config.eyebrows_joint_dir ) ),
            Radian( Degree( config.mouth_top_joint_pos * config.mouth_top_joint_dir ) ),
            Radian( Degree( config.mouth_bottom_joint_pos * config.mouth_bottom_joint_dir ) )
        };
//        std::vector<int> joint_directions = { config.eyebrows_joint_dir, config.mouth_top_joint_dir, config.mouth_bottom_joint_dir };

        multi_pub_.publish( "joint_state_command", quickdev::make_const_shared( joint_state_msg ) );

        if( config.save_params && !config_.save_params )
        {
            QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

            auto joint_names_it = last_joint_state_msg_.name.cbegin();
            auto joint_positions_it = last_joint_state_msg_.position.cbegin();
//            auto joint_directions_it = joint_directions.cbegin();

            for( ; joint_names_it != last_joint_state_msg_.name.cend(); ++joint_names_it, ++joint_positions_it )
            {
                auto joint_config_values = ros::ParamReader<XmlRpc::XmlRpcValue, 1>::getXmlRpcValue( joints_config_values_, *joint_names_it );
                auto const current_origin = ros::ParamReader<double, 1>::getXmlRpcValue( joint_config_values, "origin" );
                auto const current_direction = ros::ParamReader<int, 1>::getXmlRpcValue( joint_config_values, "direction" );

                nh_rel.setParam( "joints/" + *joint_names_it + "/origin", current_origin + current_direction * Degree( Radian( *joint_positions_it ) ) );
//                nh_rel.setParam( "joints/" + *joint_names_it + "/direction", *joint_directions_it );
            }
        }
    }
};

#endif // BANDITCALIBRATIONPROTOTYPE_BANDITCALIBRATION_CALIBRATIONNODE_H_
