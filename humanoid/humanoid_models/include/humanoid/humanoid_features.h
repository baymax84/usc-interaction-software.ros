/***************************************************************************
 *  include/humanoid/humanoid_features.h
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

#ifndef HUMANOIDMODELS_HUMANOID_HUMANOIDFEATURES_H_
#define HUMANOIDMODELS_HUMANOID_HUMANOIDFEATURES_H_

#include <vector>
#include <string>
#include <array>

#include <quickdev/math.h>
#include <quickdev/message_array_cache.h>
#include <quickdev/geometry_message_conversions.h>
#include <quickdev/numeric_unit_conversions.h>
#include <quickdev/feature.h>
#include <angles/angles.h>
#include <quickdev/feature_with_covariance.h>
#include <humanoid/soft_classification.h>

#include <humanoid_models/HumanoidStateArray.h>
#include <humanoid_models/JointStateArray.h>

#include <kdl/jntarray.hpp>

namespace humanoid
{

typedef std::string _JointName;
typedef std::vector<_JointName> _JointNames;
typedef std::map<_JointName, _JointName> _JointDependencyMap;
typedef std::map<_JointName, std::vector<std::vector<int> > > _JointStateMap;
typedef geometry_msgs::PoseWithCovariance _PoseWithConfidenceMsg;
typedef humanoid_models::HumanoidJoint _HumanoidJointMsg;
typedef humanoid_models::HumanoidState _HumanoidStateMsg;
typedef humanoid_models::HumanoidStateArray _HumanoidStateArrayMsg;
typedef sensor_msgs::JointState _JointStateMsg;
typedef humanoid_models::JointStateArray _JointStateArrayMsg;
typedef KDL::JntArray _JntArray;
typedef SoftClassification _SoftClassification;
typedef _SoftClassification::_SoftClassificationSet _SoftClassificationSet;

static _JointNames const JOINT_NAMES_
{
    "head",             // 0  : nnn :
    "neck",             // 1  : ryp :
    "torso",            // 2  : ryp :
    "pelvis",           // 3  : ryp
    "right_collar",     // 4  : nnn
    "right_shoulder",   // 5  : ryn
    "right_elbow",      // 6  : nyp
    "right_wrist",      // 7  : ryp
    "right_hand",       // 8  : nnn
    "right_finger_tip", // 9  : nnn
    "left_collar",      // 10 : nnn
    "left_shoulder",    // 11 : ryn
    "left_elbow",       // 12 : nyp
    "left_wrist",       // 13 : ryp
    "left_hand",        // 14 : nnn
    "left_finger_tip",  // 15 : nnn
    "right_hip",        // 16 : ryp
    "right_knee",       // 17 : nnp
    "right_ankle",      // 18 : ryp
    "right_foot",       // 19 : nnn
    "left_hip",         // 20 : ryp
    "left_knee",        // 21 : nnp
    "left_ankle",       // 22 : ryp
    "left_foot"         // 23 : nnn
};

namespace RotationAxes
{
    static unsigned int const NONE = 0;
    static unsigned int const CORONAL = 1;
    static unsigned int const SAGITTAL = 2;
    static unsigned int const TRANSVERSE = 3;

    static unsigned int const & N = NONE;
    static unsigned int const & C = CORONAL;
    static unsigned int const & S = SAGITTAL;
    static unsigned int const & T = TRANSVERSE;

    static unsigned int const & R = CORONAL;
    static unsigned int const & P = SAGITTAL;
    static unsigned int const & Y = TRANSVERSE;

    // "map" from plane code to plane name
    static std::vector<std::string> const names { "none", "roll", "pitch", "yaw" };
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                            //                                             //
//            [undirected graph]              //              [dependency tree]              //
//                                            //                                             //
///////////////////////////////////////////////////////////////////////////////////////////////
//                                            //                                             //
//                  [head]                    //                  [<sensor>]                 //
//                     |                      //                      ^                      //
//                  [neck]                    //                  [pelvis]                   //
//                     |                      //                      ^                      //
//                     |                      //              ______/ | \______              //
//     [left_collar]   |   [right_collar]     //             /        |        \             //
//            |     \  |  /      |            //       [left_hip]     |     [right_hip]      //
//   [left_shoulder] \ | / [right_shoulder]   //            ^         |          ^           //
//            |     [torso]      |            //      [left_knee]     |     [right_knee]     //
//      [left_elbow]   |   [right_elbow]      //            ^         |          ^           //
//            |        |         |            //     [left_ankle]     |     [right_ankle]    //
//      [left_wrist]   |   [right_wrist]      //            ^      [torso]       ^           //
//            |        |         |            //      [left_foot]     ^     [right_foot]     //
//       [left_hand]   |   [right_hand]       //             _______/ | \_______________     //
//            |        |         |            //            /          \                \    //
// [left_finger_tip]   |   [right_finger_tip] //     [left_collar]  [right_collar]    [neck] //
//                 [pelvis]                   //           ^              ^              ^   //
//                  /     \                   //   [left_shoulder]  [right_shoulder]  [head] //
//                 /       \                  //           ^              ^                  //
//         [left_hip]     [right_hip]         //      [left_elbow]  [right_elbow]            //
//              |             |               //           ^              ^                  //
//        [left_knee]     [right_knee]        //      [left_wrist]  [right_wrist]            //
//              |             |               //           ^              ^                  //
//       [left_ankle]     [right_ankle]       //       [left_hand]  [right_hand]             //
//              |             |               //           ^              ^                  //
//        [left_foot]     [right_foot]        // [left_finger_tip]  [right_finger_tip]       //
//                                            //                                             //
///////////////////////////////////////////////////////////////////////////////////////////////

static _JointDependencyMap generateJointDependencyMap()
{
    _JointDependencyMap joint_dependency_map;
    //        parent of ["joint1"]           = "joint2"
    joint_dependency_map["head"]             = "neck";
    joint_dependency_map["neck"]             = "torso";
    joint_dependency_map["torso"]            = "pelvis";
    joint_dependency_map["pelvis"]           = "sensor";

    joint_dependency_map["right_collar"]     = "torso";
    joint_dependency_map["right_shoulder"]   = "right_collar";
    joint_dependency_map["right_elbow"]      = "right_shoulder";
    joint_dependency_map["right_wrist"]      = "right_elbow";
    joint_dependency_map["right_hand"]       = "right_wrist";
    joint_dependency_map["right_finger_tip"] = "right_hand";

    joint_dependency_map["right_hip"]        = "pelvis";
    joint_dependency_map["right_knee"]       = "right_hip";
    joint_dependency_map["right_ankle"]      = "right_knee";
    joint_dependency_map["right_foot"]       = "right_ankle";

    joint_dependency_map["left_collar"]      = "torso";
    joint_dependency_map["left_shoulder"]    = "left_collar";
    joint_dependency_map["left_elbow"]       = "left_shoulder";
    joint_dependency_map["left_wrist"]       = "left_elbow";
    joint_dependency_map["left_hand"]        = "left_wrist";
    joint_dependency_map["left_finger_tip"]  = "left_hand";

    joint_dependency_map["left_hip"]         = "pelvis";
    joint_dependency_map["left_knee"]        = "left_hip";
    joint_dependency_map["left_ankle"]       = "left_knee";
    joint_dependency_map["left_foot"]        = "left_ankle";

    return joint_dependency_map;
}

static auto getJointDependencyMap() -> decltype( generateJointDependencyMap() ) const &
{
    static auto const & joint_dependency_map = generateJointDependencyMap();

    return joint_dependency_map;
}

static _JointStateMap generateJointStateMap()
{
    // map from [name]{ r, p, y } -> { plane_code, plane_code, plane_code }
    _JointStateMap joint_state_map;

    auto const & n = RotationAxes::NONE;
    //auto const & r = AnatomicalPlanes::CORONAL;
    //auto const & p = AnatomicalPlanes::SAGITTAL;
    //auto const & y = AnatomicalPlanes::TRANSVERSE;
    auto const & x = RotationAxes::CORONAL;
    auto const & y = RotationAxes::SAGITTAL;
    auto const & z = RotationAxes::TRANSVERSE;

    //                                       x   y   z      x  y  z
    //                                       r   p   y      r  p  y
    joint_state_map[JOINT_NAMES_[0]]  = { {  n,  n,  n }, { 0, 0, 0   } }; // head
    joint_state_map[JOINT_NAMES_[1]]  = { { -z, -x,  y }, { 0, 0, 180 } }; // neck
    joint_state_map[JOINT_NAMES_[2]]  = { { -z, -x,  y }, { 0, 0, 180 } }; // torso
    joint_state_map[JOINT_NAMES_[3]]  = { { -z, -x,  y }, { 0, 0, 180 } }; // pelvis
    joint_state_map[JOINT_NAMES_[4]]  = { {  n,  n,  n }, { 0, 0, 0   } }; // right_collar
    joint_state_map[JOINT_NAMES_[5]]  = { { -z,  n, -y }, { 0, 0, 180 } }; // right_shoulder
    joint_state_map[JOINT_NAMES_[6]]  = { {  n, -z,  y }, { 0, 0, 90  } }; // right_elbow
    joint_state_map[JOINT_NAMES_[7]]  = { {  n,  n,  n }, { 0, 0, 0   } }; // right_wrist
    joint_state_map[JOINT_NAMES_[8]]  = { {  n,  n,  n }, { 0, 0, 0   } }; // right_hand
    joint_state_map[JOINT_NAMES_[9]]  = { {  n,  n,  n }, { 0, 0, 0   } }; // right_finger_tip
    joint_state_map[JOINT_NAMES_[10]] = { {  n,  n,  n }, { 0, 0, 0   } }; // left_collar
    joint_state_map[JOINT_NAMES_[11]] = { { -z,  n,  y }, { 0, 0, 180 } }; // left_shoulder
    joint_state_map[JOINT_NAMES_[12]] = { {  n,  z,  y }, { 0, 0, -90 } }; // left_elbow
    joint_state_map[JOINT_NAMES_[13]] = { {  n,  n,  n }, { 0, 0, 0   } }; // left_wrist
    joint_state_map[JOINT_NAMES_[14]] = { {  n,  n,  n }, { 0, 0, 0   } }; // left_hand
    joint_state_map[JOINT_NAMES_[15]] = { {  n,  n,  n }, { 0, 0, 0   } }; // left_finger_tip
    joint_state_map[JOINT_NAMES_[16]] = { { -z, -x,  n }, { 0, 0, 180 } }; // right_hip
    joint_state_map[JOINT_NAMES_[17]] = { {  n, -x,  y }, { 0, 0, 180 } }; // right_knee
    joint_state_map[JOINT_NAMES_[18]] = { { -z, -x,  y }, { 0, 0, 180 } }; // right_ankle
    joint_state_map[JOINT_NAMES_[19]] = { {  n,  n,  n }, { 0, 0, 0   } }; // right_foot
    joint_state_map[JOINT_NAMES_[20]] = { { -z, -x,  n }, { 0, 0, 180 } }; // left_hip
    joint_state_map[JOINT_NAMES_[21]] = { {  n, -x,  y }, { 0, 0, 180 } }; // left_knee
    joint_state_map[JOINT_NAMES_[22]] = { { -z, -x,  y }, { 0, 0, 180 } }; // left_ankle
    joint_state_map[JOINT_NAMES_[23]] = { {  n,  n,  n }, { 0, 0, 0   } }; // left_foot

    return joint_state_map;
}

static auto getJointStateMap() -> decltype( generateJointStateMap() ) const &
{
    static auto const & joint_state_map = generateJointStateMap();

    return joint_state_map;
}

/*
struct HumanoidChainIterator : std::iterator<std::bidirectional_iterator_tag,
{

};
*/

class Humanoid : public quickdev::NamedMessageArrayCache<_HumanoidJointMsg>//, public quickdev::TimedMessageArrayCache<_HumanoidJointMsg>
{
public:
    typedef quickdev::NamedMessageArrayCache<_HumanoidJointMsg> _NamedMessageArrayCache;
    typedef quickdev::TimedMessageArrayCache<_HumanoidJointMsg> _TimedMessageArrayCache;

    typedef double _JointFeature;
    typedef quickdev::Feature<_JointFeature> _HumanoidFeature;

    struct Header
    {
        ros::Time stamp;
    };

    std::string name;
    Header header;

protected:
    _HumanoidStateMsg joint_array_msg_;

public:
    Humanoid( Humanoid const & other )
    :
        _NamedMessageArrayCache( other.getNamedMessageArrayCache() )
    {
        name = other.name;
        header = other.header;
    }

    Humanoid( const _HumanoidStateMsg & msg = _HumanoidStateMsg() )
    :
        _NamedMessageArrayCache()
//        _TimedMessageArrayCache( _NamedMessageArrayCache::getStorage() )
    {
//        quickdev::start_stream_indented( "updating joints from Humanoid::Humanoid()" );
        updateJoints( msg );
//        quickdev::end_stream_indented();
    }

    _NamedMessageArrayCache & getNamedMessageArrayCache()
    {
        return _NamedMessageArrayCache::getInstance();
    }

    _NamedMessageArrayCache const & getNamedMessageArrayCache() const
    {
        return _NamedMessageArrayCache::getInstance();
    }

    Humanoid & updateJoints( const _HumanoidStateMsg & msg )
    {
        name = msg.name;
        header.stamp = msg.header.stamp;

//        quickdev::start_stream_indented( "updating humanoid joints in Humanoid::updateJoints()" );
        _NamedMessageArrayCache::updateMessages( msg.joints );
//        quickdev::end_stream_indented();

        return *this;
    }

    void eraseOld( double const & message_lifetime = 1.0 )
    {
//        _TimedMessageArrayCache::eraseOld( message_lifetime );
    }

    Humanoid & operator=( const _HumanoidStateMsg & msg )
    {
//        quickdev::start_stream_indented( "updating joints from Humanoid::operator=()" );
        return updateJoints( msg );
//        quickdev::end_stream_indented();
    }

    _HumanoidStateMsg const & getJointsMessage()
    {
        // triggers re-building of the cached ROS message, if necessary
        joint_array_msg_.joints = _NamedMessageArrayCache::getMessages();
        joint_array_msg_.header.stamp = ros::Time::now(); //_TimedMessageArrayCache::getStamp();
        joint_array_msg_.name = name;
        return joint_array_msg_;
    }

    _JointStateMsg getJointStateMessage( _JointStateMap const & joint_state_map = getJointStateMap() ) const
    {
        _JointStateMsg joint_state_msg;

        // ensure we have at most 3 * number_of_joints to work with (since we'll be making a "joint state" for every rotational component of each joint )
        auto const num_joint_states = _NamedMessageArrayCache::size() * 3;

        joint_state_msg.name.reserve( num_joint_states );
        joint_state_msg.position.reserve( num_joint_states );

        joint_state_msg.velocity.resize( num_joint_states );
        joint_state_msg.effort.resize( num_joint_states );

        //auto const & joint_state_map = getJointStateMap();

        for( auto joint = _NamedMessageArrayCache::cbegin(); joint != _NamedMessageArrayCache::cend(); ++joint )
        {
            // {
            //   { [0:3], [0:3], [0:3] },
            //   { [-180:180], [-180:180], [-180:180] }
            // }
            auto const & joint_state_entry = joint_state_map.find(joint->name)->second;
            // convert: btVector3 <- btQuaternion <- geometry_msgs::Quaternion
            btVector3 const position_vec = unit::make_unit( unit::convert<btQuaternion>( joint->pose.pose.orientation ) );

            // a joint state entry stores a plane ID and an offset for each component of rotation
            auto const & plane_ids = joint_state_entry.at( 0 );
            auto const & offsets = joint_state_entry.at( 1 );

            for( size_t i = 0; i < 3; ++i )
            {
                auto const & rotation_axis_raw = plane_ids.at( i );
                auto const rotation_axis = abs( rotation_axis_raw );

                // if the given axis has no mapping to an anatomical plane, just skip it
                if( abs( rotation_axis ) == RotationAxes::NONE ) continue;

                auto const rotation_axis_sign = rotation_axis_raw < 0 ? -1 : 1;

                auto const & offset_degrees = offsets.at( i );
                double const offset_rad = Radian( Degree( offset_degrees ) );

                joint_state_msg.name.push_back( joint->name + "_" + RotationAxes::names.at( i + 1 ) );
                joint_state_msg.position.push_back( rotation_axis_sign * position_vec.m_floats[rotation_axis - 1] + offset_rad );
            }

            /*for( auto component_id = joint_state_entry.cbegin(); component_id != joint_state_entry.cend(); ++component_id )
            {
                if( *component_id == 0 ) continue;

                joint_state_msg.name.push_back( joint->name + "_" + AnatomicalPlanes::names[*component_id] );
                joint_state_msg.position.push_back( position_vec.m_floats[*component_id - 1] );
            }*/
        }

        return joint_state_msg;
    }
/*
    _JointStateMsg getJointStateChain( std::string const & from_joint, std::string const & to_joint, _JointStateMap const & joint_state_map = getJointStateMap() ) const
    {
        _JointStateMsg joint_state_msg;

        // ensure we have at most 3 * number_of_joints to work with (since we'll be making a "joint state" for every rotational component of each joint )
        auto const num_joint_states = _NamedMessageArrayCache::size() * 3;

        joint_state_msg.name.reserve( num_joint_states );
        joint_state_msg.position.reserve( num_joint_states );

        joint_state_msg.velocity.resize( num_joint_states );
        joint_state_msg.effort.resize( num_joint_states );

        //auto const & joint_state_map = getJointStateMap();

        for( auto joint = find( to_joint ); joint != end(); find( getParentJointName( joint.name ) )
        {
            // {
            //   { [0:3], [0:3], [0:3] },
            //   { [-180:180], [-180:180], [-180:180] }
            // }
            auto const & joint_state_entry = joint_state_map.find(joint->name)->second;
            // convert: btVector3 <- btQuaternion <- geometry_msgs::Quaternion
            btVector3 const position_vec = unit::make_unit( unit::convert<btQuaternion>( joint->pose.pose.orientation ) );

            // a joint state entry stores a plane ID and an offset for each component of rotation
            auto const & plane_ids = joint_state_entry.at( 0 );
            auto const & offsets = joint_state_entry.at( 1 );

            for( size_t i = 0; i < 3; ++i )
            {
                auto const & rotation_axis_raw = plane_ids.at( i );
                auto const rotation_axis = abs( rotation_axis_raw );

                // if the given axis has no mapping to an anatomical plane, just skip it
                if( abs( rotation_axis ) == RotationAxes::NONE ) continue;

                auto const rotation_axis_sign = rotation_axis_raw < 0 ? -1 : 1;

                auto const & offset_degrees = offsets.at( i );
                double const offset_rad = Radian( Degree( offset_degrees ) );

                joint_state_msg.name.push_back( joint->name + "_" + RotationAxes::names.at( i + 1 ) );
                joint_state_msg.position.push_back( rotation_axis_sign * position_vec.m_floats[rotation_axis - 1] + offset_rad );
            }
//
            for( auto component_id = joint_state_entry.cbegin(); component_id != joint_state_entry.cend(); ++component_id )
            {
                if( *component_id == 0 ) continue;

                joint_state_msg.name.push_back( joint->name + "_" + AnatomicalPlanes::names[*component_id] );
                joint_state_msg.position.push_back( position_vec.m_floats[*component_id - 1] );
            }
//
        }

        return joint_state_msg;
    }
*/

    std::string getParentJointName( std::string const & joint_name ) const
    {
        auto const & joint_dependency_map = getJointDependencyMap();
        return joint_dependency_map.find(joint_name)->second;
    }

    _HumanoidJointMsg getParentJoint( _HumanoidJointMsg const & joint ) const
    {
        return operator[]( getParentJointName( joint.name ) );
    }

    _HumanoidFeature getFeature() const
    {
        auto const joint_state_msg = getJointStateMessage();

        return _HumanoidFeature( joint_state_msg.position );
    }

    const Humanoid & updateJoint( _HumanoidJointMsg const & msg )
    {
        _NamedMessageArrayCache::updateMessage( msg );

        return *this;
    }

    size_t size() const
    {
        return _NamedMessageArrayCache::size();
    }

    // use world frame
    static btTransform calculateBaseLinkTransform( _HumanoidStateMsg const & state_msg, btTransform const & world_to_sensor_tf )
    {
        // given: [sensor -> { joints }]
        //        [world -> sensor]
        // need: 2d pose of pelvis in world x-y plane, transformed back to the sensor
        // solution: position = [world -> pelvis], z = 0
        //           angle = angle formed by x-y values of ( angle of [world->pelvis] * vec( 1, 0, 0 ) )
        // result [sensor -> base_link] = -[world -> sensor] * position

        // build map from state message
        std::map<std::string, _HumanoidJointMsg> joint_map;
        for( auto joint = state_msg.joints.cbegin(); joint != state_msg.joints.cend(); ++joint )
        {
            joint_map[joint->name] = *joint;
        }

        auto const sensor_to_pelvis_tf = unit::convert<btTransform>( joint_map["pelvis"] );
        auto const world_to_pelvis_tf = world_to_sensor_tf * sensor_to_pelvis_tf;

        btTransform const angle3d_tf( world_to_pelvis_tf.getRotation() );
        btVector3 const unit_vec( 1, 0, 0 );
        btVector3 const angle3d_vec = angle3d_tf * unit_vec;
        btVector3 const angle2d_vec( angle3d_vec.getX(), angle3d_vec.getY(), 0 );
        double const yaw = atan2( angle2d_vec.getY(), angle2d_vec.getX() );

        btVector3 position = world_to_pelvis_tf.getOrigin();
        position.setZ( 0 );

        btTransform const world_to_base_link_tf( btQuaternion( yaw, 0, 0 ), position );

        return world_to_sensor_tf.inverse() * world_to_base_link_tf;
    }

    // assumes sensor has no roll with respect to the world
    static btTransform calculateBaseLinkTransform( _HumanoidStateMsg const & state_msg )
    {
        // build map from state message
        std::map<std::string, _HumanoidJointMsg> joint_map;
        for( auto joint = state_msg.joints.cbegin(); joint != state_msg.joints.cend(); ++joint )
        {
            joint_map[joint->name] = *joint;
        }

        // given: [sensor -> { joints }]
        // need: 2d pose of pelvis projected to the z-coordinate of the foot that is farthest from the pelvis
        auto const sensor_to_left_foot_tf = unit::convert<btTransform>( joint_map["left_foot"] );
        auto const sensor_to_right_foot_tf = unit::convert<btTransform>( joint_map["right_foot"] );
        auto const sensor_to_pelvis_tf = unit::convert<btTransform>( joint_map["pelvis"] );

        auto const pelvis_to_left_foot_vec = sensor_to_left_foot_tf.getOrigin() - sensor_to_pelvis_tf.getOrigin();
        auto const pelvis_to_right_foot_vec = sensor_to_right_foot_tf.getOrigin() - sensor_to_pelvis_tf.getOrigin();

        auto const sensor_to_farthest_foot_vec = fabs( pelvis_to_left_foot_vec.length() ) > fabs( pelvis_to_right_foot_vec.length() ) ? pelvis_to_left_foot_vec : pelvis_to_right_foot_vec;

        auto const sensor_to_pelvis_ori = unit::convert<btVector3>( sensor_to_pelvis_tf.getRotation() );

        return btTransform( btQuaternion( sensor_to_pelvis_ori.getZ(), 0, 0 ), btVector3( sensor_to_pelvis_tf.getOrigin().getX(), sensor_to_pelvis_tf.getOrigin().getY(), sensor_to_farthest_foot_vec.getZ() ) );
    }

    static _HumanoidStateMsg estimateExtraJoints( _HumanoidStateMsg const & state_msg )
    {
        auto const & joint_dependency_map = humanoid::getJointDependencyMap();

        // build map from state message
        std::map<std::string, _HumanoidJointMsg> joint_map;
        for( auto joint = state_msg.joints.cbegin(); joint != state_msg.joints.cend(); ++joint )
        {
            joint_map[joint->name] = *joint;
        }

        auto const now = state_msg.header.stamp;

        // pelvis
        if( joint_map.find( "pelvis" ) == joint_map.end() )
        {
            auto parent_joint_msg1_it = joint_map.find( "left_hip" );
            auto parent_joint_msg2_it = joint_map.find( "right_hip" );

            // if the parent joints don't exist, then we can't do anything here
            if( parent_joint_msg1_it != joint_map.end() && parent_joint_msg1_it != joint_map.end() )
            {
                auto & parent_joint_msg1 = parent_joint_msg1_it->second;
                auto & parent_joint_msg2 = parent_joint_msg2_it->second;

                auto const sensor_to_left_hip_tf = unit::convert<btTransform>( parent_joint_msg1 );
                auto const sensor_to_right_hip_tf = unit::convert<btTransform>( parent_joint_msg2 );

                _HumanoidJointMsg joint_msg;

                joint_msg.header = parent_joint_msg1.header;

                joint_msg.name = "pelvis";
                joint_msg.parent_name = "sensor";

                auto const sensor_to_pelvis_tf = btTransform( ( sensor_to_left_hip_tf.getRotation() + sensor_to_right_hip_tf.getRotation() ) / 2, ( sensor_to_left_hip_tf.getOrigin() + sensor_to_right_hip_tf.getOrigin() ) / 2 );

                joint_msg.pose.pose = unit::make_unit( sensor_to_pelvis_tf );

                joint_msg.pose.covariance[0 * 6 + 0] = parent_joint_msg1.pose.covariance[0 * 6 + 0] + parent_joint_msg2.pose.covariance[0 * 6 + 0];
                joint_msg.pose.covariance[1 * 6 + 1] = parent_joint_msg1.pose.covariance[1 * 6 + 1] + parent_joint_msg2.pose.covariance[1 * 6 + 1];
                joint_msg.pose.covariance[2 * 6 + 2] = parent_joint_msg1.pose.covariance[2 * 6 + 2] + parent_joint_msg2.pose.covariance[2 * 6 + 2];
                joint_msg.pose.covariance[3 * 6 + 3] = parent_joint_msg1.pose.covariance[3 * 6 + 3] + parent_joint_msg2.pose.covariance[3 * 6 + 3];
                joint_msg.pose.covariance[4 * 6 + 4] = parent_joint_msg1.pose.covariance[4 * 6 + 4] + parent_joint_msg2.pose.covariance[4 * 6 + 4];
                joint_msg.pose.covariance[5 * 6 + 5] = parent_joint_msg1.pose.covariance[5 * 6 + 5] + parent_joint_msg2.pose.covariance[5 * 6 + 5];

                joint_map[joint_msg.name] = joint_msg;
            }
        }

        // head given pelvis
        {
            auto parent_joint_msg_it = joint_map.find( "pelvis" );
            auto joint_msg_it = joint_map.find( "head" );

            // if the parent joint doesn't exist, then we can't do anything here
            if( parent_joint_msg_it != joint_map.end() && joint_msg_it != joint_map.end() )
            {
                auto & parent_joint_msg = parent_joint_msg_it->second;
                auto & joint_msg = joint_msg_it->second;

                // pelvis -> head * pelvis * head
                joint_msg.pose.covariance[5 * 6 + 5] = quickdev::gaussian_product_variance( 0.24682683, parent_joint_msg.pose.covariance[5 * 6 + 5], joint_msg.pose.covariance[5 * 6 + 5] );
            }
        }

        // eyes given head
        if( joint_map.find( "eyes" ) == joint_map.end() )
        {
            auto parent_joint_msg_it = joint_map.find( "head" );

            // if the parent joint doesn't exist, then we can't do anything here
            if( parent_joint_msg_it != joint_map.end() )
            {
                auto & parent_joint_msg = parent_joint_msg_it->second;

                _HumanoidJointMsg joint_msg;

                joint_msg.header = parent_joint_msg.header;

                joint_msg.name = "eyes";
                joint_msg.parent_name = "head";

                // copy parent pose
                joint_msg.pose.covariance = parent_joint_msg.pose.covariance;

                auto const sensor_to_head_tf = unit::convert<btTransform>( parent_joint_msg );
                // the eyes are 0.039 m along the head's x-axis
                auto const sensor_to_eyes_tf = sensor_to_head_tf * btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0.039, 0, 0 ) );

                joint_msg.pose.pose = unit::make_unit( sensor_to_eyes_tf );

                // head -> eyes * head
                joint_msg.pose.covariance[5 * 6 + 5] = quickdev::gaussian_product_variance( 0.787377587, parent_joint_msg.pose.covariance[5 * 6 + 5] );

                joint_map[joint_msg.name] = joint_msg;
            }
        }

        // ears
        if( joint_map.find( "ears" ) == joint_map.end() )
        {
            auto parent_joint_msg_it = joint_map.find( "head" );

            // if the parent joint doesn't exist, then we can't do anything here
            if( parent_joint_msg_it != joint_map.end() )
            {
                auto & parent_joint_msg = parent_joint_msg_it->second;

                auto const sensor_to_head_tf = unit::convert<btTransform>( parent_joint_msg );

                // left ear
                {
                    _HumanoidJointMsg joint_msg;

                    joint_msg.header = parent_joint_msg.header;

                    joint_msg.name = "left_ear";
                    joint_msg.parent_name = "head";

                    // copy parent pose
                    joint_msg.pose.covariance = parent_joint_msg.pose.covariance;

                    auto const head_to_left_ear_tf = sensor_to_head_tf * btTransform( btQuaternion( M_PI_2, 0, 0 ), btVector3( 0, -0.039, 0 ) );

                    joint_msg.pose.pose = unit::make_unit( head_to_left_ear_tf );

                    joint_map[joint_msg.name] = joint_msg;
                }

                // right ear
                {
                    _HumanoidJointMsg joint_msg;

                    joint_msg.header = parent_joint_msg.header;

                    joint_msg.name = "right_ear";
                    joint_msg.parent_name = "head";

                    // copy parent pose
                    joint_msg.pose.covariance = parent_joint_msg.pose.covariance;

                    // the eyes are 0.039 m along the head's x-axis
                    auto const head_to_right_ear_tf = sensor_to_head_tf * btTransform( btQuaternion( -M_PI_2, 0, 0 ), btVector3( 0, 0.039, 0 ) );

                    joint_msg.pose.pose = unit::make_unit( head_to_right_ear_tf );

                    joint_map[joint_msg.name] = joint_msg;
                }
            }
        }

        // nose
        if( joint_map.find( "nose" ) == joint_map.end() )
        {
            auto parent_joint_msg_it = joint_map.find( "head" );

            // if the parent joint doesn't exist, then we can't do anything here
            if( parent_joint_msg_it != joint_map.end() )
            {
                auto & parent_joint_msg = parent_joint_msg_it->second;

                auto const sensor_to_head_tf = unit::convert<btTransform>( parent_joint_msg );

                _HumanoidJointMsg joint_msg;

                joint_msg.header = parent_joint_msg.header;

                joint_msg.name = "nose";
                joint_msg.parent_name = "head";

                // copy parent pose
                joint_msg.pose.covariance = parent_joint_msg.pose.covariance;

                auto const head_to_nose_tf = sensor_to_head_tf * btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0.039, 0, 0 ) );

                joint_msg.pose.pose = unit::make_unit( head_to_nose_tf );

                joint_map[joint_msg.name] = joint_msg;
            }
        }

        // mouth
        if( joint_map.find( "mouth" ) == joint_map.end() )
        {
            auto parent_joint_msg_it = joint_map.find( "head" );

            // if the parent joint doesn't exist, then we can't do anything here
            if( parent_joint_msg_it != joint_map.end() )
            {
                auto & parent_joint_msg = parent_joint_msg_it->second;

                auto const sensor_to_head_tf = unit::convert<btTransform>( parent_joint_msg );

                _HumanoidJointMsg joint_msg;

                joint_msg.header = parent_joint_msg.header;

                joint_msg.name = "mouth";
                joint_msg.parent_name = "head";

                // copy parent pose
                joint_msg.pose.covariance = parent_joint_msg.pose.covariance;

                auto const head_to_mouth_tf = sensor_to_head_tf * btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0.039, 0, 0 ) );

                joint_msg.pose.pose = unit::make_unit( head_to_mouth_tf );

                joint_map[joint_msg.name] = joint_msg;
            }
        }

        _HumanoidStateMsg result;
        result.header = state_msg.header;
        result.name = state_msg.name;

        for( auto joint = joint_map.cbegin(); joint != joint_map.cend(); ++joint )
        {
            result.joints.push_back( joint->second );
        }

        return result;
    }

    _HumanoidStateMsg estimateExtraJoints()
    {
        return estimateExtraJoints( getJointsMessage() );
    }
};

typedef Humanoid _Humanoid;
typedef std::pair<_Humanoid, _Humanoid> _HumanoidPair;

class HumanoidFeatureRecognizer
{
public:
    _Humanoid humanoid_;

    HumanoidFeatureRecognizer( _Humanoid const & humanoid = _Humanoid() )
    :
        humanoid_( humanoid )
    {
        //
    }

    std::pair<std::string, std::string> getClosestPair( HumanoidFeatureRecognizer const & other )
    {
        double closest_distance = std::numeric_limits<double>::max();
        std::string joint1_name;
        std::string joint2_name;

        for( auto joint1_it = humanoid_.cbegin(); joint1_it != humanoid_.cend(); ++joint1_it )
        {
            for( auto joint2_it = other.humanoid_.cbegin(); joint2_it != other.humanoid_.cend(); ++joint2_it )
            {
                auto const & joint1 = *joint1_it;
                auto const & joint2 = *joint2_it;

                double const distance = unit::convert<btVector3>( joint1 ).distance( unit::convert<btVector3>( joint2 ) );
                if( distance < closest_distance )
                {
                    closest_distance = distance;
                    joint1_name = joint1.name;
                    joint2_name = joint2.name;
                }
            }
        }

        return std::pair<std::string, std::string>( joint1_name, joint2_name );
    }

    // find the closest joint in the other feature to our <joint1_name>
    std::string getClosestJoint( std::string const & joint1_name, HumanoidFeatureRecognizer const & other )
    {
        double closest_distance = std::numeric_limits<double>::max();
        std::string joint2_name;

        auto joint1_it = humanoid_.find( joint1_name );
        if( joint1_it == humanoid_.end() ) return "";

        for( auto joint2_it = other.humanoid_.cbegin(); joint2_it != other.humanoid_.cend(); ++joint2_it )
        {
            auto const & joint1 = *joint1_it;
            auto const & joint2 = *joint2_it;

            double const distance = unit::convert<btVector3>( joint1 ).distance( unit::convert<btVector3>( joint2 ) );
            if( distance < closest_distance )
            {
                closest_distance = distance;
                joint2_name = joint2.name;
            }
        }

        return joint2_name;
    }

    quickdev::FeatureWithCovariance<btVector3, 3> getOrigin( std::string const & joint_name = "pelvis" ) const
    {
        auto const & joint = humanoid_[joint_name];

        quickdev::FeatureWithCovariance<btVector3, 3> result( unit::convert<btVector3>( joint ) );

        result.getCovariance()( 0, 0 ) = joint.pose.covariance[0 * 6 + 0];
        result.getCovariance()( 1, 1 ) = joint.pose.covariance[1 * 6 + 1];
        result.getCovariance()( 2, 2 ) = joint.pose.covariance[2 * 6 + 2];

        return result;
    }

    quickdev::FeatureWithCovariance<double, 1> getRotation( std::string const & joint_name = "pelvis" ) const
    {
        auto const & joint = humanoid_[joint_name];

        quickdev::FeatureWithCovariance<double, 1> result( unit::convert<btVector3>( unit::convert<btQuaternion>( humanoid_[joint_name] ) ).getZ() );

        result.getCovariance()( 0, 0 ) = joint.pose.covariance[5 * 6 + 5];

        return result;
    }
};

} // humanoid

//! conversion: _HumanoidJointMsg -> btVector3
DECLARE_UNIT_CONVERSION_LAMBDA( humanoid::_HumanoidJointMsg, btVector3, msg, return btVector3( msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z ); )
//! conversion: _HumanoidJointMsg -> btQuaternion
DECLARE_UNIT_CONVERSION_LAMBDA( humanoid::_HumanoidJointMsg, btQuaternion, msg, return btQuaternion( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ); )
//! conversion: _HumanoidJointMsg -> btTransform
DECLARE_UNIT_CONVERSION_LAMBDA( humanoid::_HumanoidJointMsg, btTransform, msg, return btTransform( btQuaternion( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ), btVector3( msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z ) ); )
//! conversion: _JointStateMsg -> KDL::JntArray
//DECLARE_UNIT_CONVERSION_LAMBDA( humanoid::_JointStateMsg, humanoid::_JntArray, msg, humanoid::_JntArray joints( msg.positions.size() ); std::copy( &joints( 0 ), &joints( joints.rows() - 1 ), msg.positions.begin() ); return joints; )

#endif // HUMANOIDMODELS_HUMANOID_HUMANOIDFEATURES_H_
