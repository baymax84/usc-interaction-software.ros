/***************************************************************************
 *  include/humanoid/virtual_humanoid_node.h
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

#ifndef HUMANOIDMODELS_HUMANOID_VIRTUALHUMANOIDNODE_H_
#define HUMANOIDMODELS_HUMANOID_VIRTUALHUMANOIDNODE_H_

#include <quickdev/node.h>
#include <quickdev/reconfigure_policy.h>
#include <quickdev/tf_tranceiver_policy.h>

#include <quickdev/multi_publisher.h>

#include <humanoid/humanoid_features.h>

#include <humanoid_models/VirtualHumanoidConfig.h>

#include <array>

typedef humanoid_models::VirtualHumanoidConfig _VirtualHumanoidConfig;
typedef quickdev::ReconfigurePolicy<_VirtualHumanoidConfig> _VirtualHumanoidLiveParams;
typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

using humanoid::_HumanoidStateArrayMsg;
using humanoid::_HumanoidStateMsg;
using humanoid::_HumanoidJointMsg;
using humanoid::_JointName;

//! A node that can publish tf frames and humanoid state messages for a virtual humanoid
QUICKDEV_DECLARE_NODE( VirtualHumanoid, _VirtualHumanoidLiveParams, _TfTranceiverPolicy )

QUICKDEV_DECLARE_NODE_CLASS( VirtualHumanoid )
{
    typedef std::map<_JointName, btVector3> _VirtualHumanoidJointRelPosMap;
    typedef std::map<_JointName, std::vector<double *> > _VirtualHumanoidJointRelOriMap;

    ros::MultiPublisher<> multi_pub_;
    double zero_angle_;
    _VirtualHumanoidJointRelOriMap const rel_ori_map_;

    // regex to match any std::make_pair(...) multi-line block below:
    // std::make_pair(\s)(\s+)\($\s+("\w+"),$\s+.+$\s+\{$\s+(&\S+).*$\s+(&\S+).*$\s+(&\S+).*$\s+\}$\s+\)(.*)
    // \1 = newline; \2 = indentation; \3 = key; \4 = ori1; \5 = ori2; \6 = ori3; \7 = (<block separator>)
    //
    // regex to match any std::make_pair(...) single-line block below:
    // std::make_pair\(\s+("\S+"),\s+.+\{\s+(&\S+),\s+(&\S+),\s+(&\S+)\s+\}\s+\)(.*)
    // \1 = key; \2 = ori1; \3 = ori2; \4 = ori3; \5 = (<block separator>)
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( VirtualHumanoid ),
        zero_angle_( 0 ),
        rel_ori_map_
        ( {
            std::make_pair( "head",             _VirtualHumanoidJointRelOriMap::mapped_type{ &zero_angle_,                &zero_angle_,               &zero_angle_                 } ),
            std::make_pair( "neck",             _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.neck_yaw,           &config_.neck_pitch,        &config_.neck_roll           } ),
            std::make_pair( "torso",            _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.torso_yaw,          &config_.torso_pitch,       &config_.torso_roll          } ),
            std::make_pair( "waist",            _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.waist_yaw,          &config_.waist_pitch,       &config_.waist_roll          } ),
            std::make_pair( "right_collar",     _VirtualHumanoidJointRelOriMap::mapped_type{ &zero_angle_,                &zero_angle_,               &zero_angle_                 } ),
            std::make_pair( "right_shoulder",   _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.right_shoulder_yaw, &zero_angle_,               &config_.right_shoulder_roll } ),
            std::make_pair( "right_elbow",      _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.right_elbow_yaw,    &config_.right_elbow_pitch, &zero_angle_                 } ),
            std::make_pair( "right_wrist",      _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.right_wrist_yaw,    &config_.right_wrist_pitch, &config_.right_wrist_roll    } ),
            std::make_pair( "right_hand",       _VirtualHumanoidJointRelOriMap::mapped_type{ &zero_angle_,                &zero_angle_,               &zero_angle_                 } ),
            std::make_pair( "right_finger_tip", _VirtualHumanoidJointRelOriMap::mapped_type{ &zero_angle_,                &zero_angle_,               &zero_angle_                 } ),
            std::make_pair( "right_hip",        _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.right_hip_yaw,      &config_.right_hip_pitch,   &config_.right_hip_roll      } ),
            std::make_pair( "right_knee",       _VirtualHumanoidJointRelOriMap::mapped_type{ &zero_angle_,                &config_.right_knee_pitch,  &zero_angle_                 } ),
            std::make_pair( "right_ankle",      _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.right_ankle_yaw,    &config_.right_ankle_pitch, &config_.right_ankle_roll    } ),
            std::make_pair( "right_foot",       _VirtualHumanoidJointRelOriMap::mapped_type{ &zero_angle_,                &zero_angle_,               &zero_angle_                 } ),
            std::make_pair( "left_collar",      _VirtualHumanoidJointRelOriMap::mapped_type{ &zero_angle_,                &zero_angle_,               &zero_angle_                 } ),
            std::make_pair( "left_shoulder",    _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.left_shoulder_yaw,  &zero_angle_,               &config_.left_shoulder_roll  } ),
            std::make_pair( "left_elbow",       _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.left_elbow_yaw,     &config_.left_elbow_pitch,  &zero_angle_                 } ),
            std::make_pair( "left_wrist",       _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.left_wrist_yaw,     &config_.left_wrist_pitch,  &config_.left_wrist_roll     } ),
            std::make_pair( "left_hand",        _VirtualHumanoidJointRelOriMap::mapped_type{ &zero_angle_,                &zero_angle_,               &zero_angle_                 } ),
            std::make_pair( "left_finger_tip",  _VirtualHumanoidJointRelOriMap::mapped_type{ &zero_angle_,                &zero_angle_,               &zero_angle_                 } ),
            std::make_pair( "left_hip",         _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.left_hip_yaw,       &config_.left_hip_pitch,    &config_.left_hip_roll       } ),
            std::make_pair( "left_knee",        _VirtualHumanoidJointRelOriMap::mapped_type{ &zero_angle_,                &config_.left_knee_pitch,   &zero_angle_                 } ),
            std::make_pair( "left_ankle",       _VirtualHumanoidJointRelOriMap::mapped_type{ &config_.left_ankle_yaw,     &config_.left_ankle_pitch,  &config_.left_ankle_roll     } ),
            std::make_pair( "left_foot",        _VirtualHumanoidJointRelOriMap::mapped_type{ &zero_angle_,                &zero_angle_,               &zero_angle_                 } )
        } )
    {

    }

    QUICKDEV_SPIN_FIRST()
    {
        initPolicies<quickdev::policy::ALL>();

        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );
        multi_pub_.addPublishers<_HumanoidStateArrayMsg>( nh_rel, { "humanoid_states" } );
    }

    static _VirtualHumanoidJointRelPosMap generateVirtualHumanoidJointRelPosMap()
    {
        _VirtualHumanoidJointRelPosMap virtual_humanoid_joint_rel_pos_map;

        //                                ["parent->child"]
        virtual_humanoid_joint_rel_pos_map["neck->head"]                   = btVector3( 0,     0,     0.15 );
        virtual_humanoid_joint_rel_pos_map["torso->neck"]                  = btVector3( 0,     0,     0.4  );
        virtual_humanoid_joint_rel_pos_map["waist->torso"]                 = btVector3( 0,     0,     0.2  );
        virtual_humanoid_joint_rel_pos_map["sensor->waist"]                = btVector3( 0,     0,     0    );
        virtual_humanoid_joint_rel_pos_map["torso->right_collar"]          = btVector3( 0,    -0.15,  0.3  );
        virtual_humanoid_joint_rel_pos_map["right_collar->right_shoulder"] = btVector3( 0,     0,     0    );
        virtual_humanoid_joint_rel_pos_map["right_shoulder->right_elbow"]  = btVector3( 0,    -0.3,   0    );
        virtual_humanoid_joint_rel_pos_map["right_elbow->right_wrist"]     = btVector3( 0.3,   0,     0    );
        virtual_humanoid_joint_rel_pos_map["right_wrist->right_hand"]      = btVector3( 0.03,  0,     0    );
        virtual_humanoid_joint_rel_pos_map["right_hand->right_finger_tip"] = btVector3( 0.03,  0,     0    );
        virtual_humanoid_joint_rel_pos_map["torso->left_collar"]           = btVector3( 0,     0.15,  0.3  );
        virtual_humanoid_joint_rel_pos_map["left_collar->left_shoulder"]   = btVector3( 0,     0,     0    );
        virtual_humanoid_joint_rel_pos_map["left_shoulder->left_elbow"]    = btVector3( 0,     0.2,   0    );
        virtual_humanoid_joint_rel_pos_map["left_elbow->left_wrist"]       = btVector3( 0.3,   0,     0    );
        virtual_humanoid_joint_rel_pos_map["left_wrist->left_hand"]        = btVector3( 0.03,  0,     0    );
        virtual_humanoid_joint_rel_pos_map["left_hand->left_finger_tip"]   = btVector3( 0.03,  0,     0    );
        virtual_humanoid_joint_rel_pos_map["waist->right_hip"]             = btVector3( 0,    -0.15,  0    );
        virtual_humanoid_joint_rel_pos_map["right_hip->right_knee"]        = btVector3( 0,     0,    -0.4  );
        virtual_humanoid_joint_rel_pos_map["right_knee->right_ankle"]      = btVector3( 0,     0,    -0.6  );
        virtual_humanoid_joint_rel_pos_map["right_ankle->right_foot"]      = btVector3( 0.06,  0,    -0.03 );
        virtual_humanoid_joint_rel_pos_map["waist->left_hip"]              = btVector3( 0,     0.15,  0    );
        virtual_humanoid_joint_rel_pos_map["left_hip->left_knee"]          = btVector3( 0,     0,    -0.4  );
        virtual_humanoid_joint_rel_pos_map["left_knee->left_ankle"]        = btVector3( 0,     0,    -0.6  );
        virtual_humanoid_joint_rel_pos_map["left_ankle->left_foot"]        = btVector3( 0.06,  0,    -0.03 );

        return virtual_humanoid_joint_rel_pos_map;
    }

    static auto getVirtualHumanoidJointRelPosMap() -> decltype( generateVirtualHumanoidJointRelPosMap() ) const &
    {
        static auto && virtual_humanoid_joint_rel_pos_map = generateVirtualHumanoidJointRelPosMap();

        return virtual_humanoid_joint_rel_pos_map;
    }

    QUICKDEV_SPIN_ONCE()
    {
        auto const & virtual_humanoid_joint_rel_pos_map = getVirtualHumanoidJointRelPosMap();
        auto const & joint_dependency_map = humanoid::getJointDependencyMap();
        auto const & sensor_frame_name = config_.sensor_frame_name;
        auto const & user_name = config_.name;

        for( auto joint_entry = joint_dependency_map.cbegin(); joint_entry != joint_dependency_map.cend(); ++joint_entry )
        {
            bool const is_sensor_frame = joint_entry->second == "sensor";

            auto const & from_joint_name = joint_entry->second;
            auto const & to_joint_name = joint_entry->first;

            auto const from_frame_name = is_sensor_frame ? sensor_frame_name : "/" + user_name + "/" + from_joint_name;
            auto const to_frame_name = "/" + user_name + "/" + to_joint_name;

            btVector3 const & joint_rel_pos = virtual_humanoid_joint_rel_pos_map.find( from_joint_name + "->" + to_joint_name )->second;

            //printf( "looking up rotation for joint %s\n", to_joint_name.c_str() );

            auto const & ori_map_entry = rel_ori_map_.find( to_joint_name )->second;
            btQuaternion const joint_rel_ori = btQuaternion( Radian( Degree( *ori_map_entry[0] ) ), Radian( Degree( *ori_map_entry[1] ) ), Radian( Degree( *ori_map_entry[2] ) ) );

            btTransform transform( joint_rel_ori, joint_rel_pos );

            publishTransform( transform, from_frame_name, to_frame_name );
        }
    }
};

#endif // HUMANOIDMODELS_HUMANOID_VIRTUALHUMANOIDNODE_H_
