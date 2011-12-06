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
#include <quickdev/message_array_cache.h>

#include <humanoid/humanoid_features.h>

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

    typedef humanoid::_HumanoidStateMsg _HumanoidStateMsg;
    typedef humanoid::_HumanoidStateArrayMsg _HumanoidStateArrayMsg;
    typedef humanoid::_HumanoidJointMsg _HumanoidJointMsg;

    typedef humanoid::_Humanoid _Humanoid;
    //typedef std::map<std::string, _Humanoid > _HumanoidsMap;
    typedef quickdev::NamedMessageArrayCache<_Humanoid> _NamedHumanoidCache;
    typedef quickdev::TimedMessageArrayCache<_Humanoid> _TimedHumanoidCache;

    typedef humanoid::_HumanoidPair _HumanoidPair;
    typedef std::map<std::string, size_t> _HumanoidPairIdsMap;
    typedef std::vector<_HumanoidPair> _HumanoidPairs;

protected:
    quickdev::MessageCache<_HumanoidStateArrayMsg> states_cache_;



    //_HumanoidsMap humanoids_map_;
    _NamedHumanoidCache named_humanoid_cache_;
    _TimedHumanoidCache timed_humanoid_cache_;

    _HumanoidPairIdsMap humanoid_pair_ids_map_;
    _HumanoidPairs humanoid_pairs_;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( HumanoidRecognizer ),
        named_humanoid_cache_(),
        timed_humanoid_cache_( named_humanoid_cache_.getStorage() ),
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

    template<class... __ChildArgs>
    void eraseOld( __ChildArgs&&... child_args )
    {
        // remove old frames
        timed_humanoid_cache_.eraseOld( child_args... );
    }

    /*void notifyErase( const typename _MessageArray::iterator & msg )
    {
        const auto & erase_index = message_index_map_.find( msg->name );
        // starting at the item to erase, decrement all indices by 1
        for( auto message_index = erase_index; message_index != message_index_map_.end(); ++message_index )
        {
            --message_index->second;
        }

        // erase the indicated item from our index map
        message_index_map_.erase( erase_index );
    }*/

    QUICKDEV_DECLARE_ACCESSOR2( named_humanoid_cache_, Humanoids )
    QUICKDEV_DECLARE_ACCESSOR2( humanoid_pairs_, HumanoidPairs )

    /*auto getHumanoids() -> decltype( named_humanoid_cache_ ) &
    {
        return named_humanoid_cache_;
    }

    auto getHumanoidPairs() -> decltype( humanoid_pairs_ ) &
    {
        return humanoid_pairs_;
    }*/

    //! Build a list of unique humanoids (mapped from name to _Humanoid)
    void updateHumanoids( const _HumanoidStateArrayMsg::ConstPtr & states_msg )
    {
        // make sure the given message isn't null
        if( !states_msg ) return;

        for( auto humanoid_msg = states_msg->states.begin(); humanoid_msg != states_msg->states.end(); ++humanoid_msg )
        {
            named_humanoid_cache_.updateMessage( _Humanoid( *humanoid_msg ) );
        }
    }

    //! Build a list of unique humanoid pairs (mapped from combined name to _HumanoidPair)
    /*! \note buildStatesMap needs to be called first; calling buildStatesMap invalidates all data generated by this function */
    void updateHumanoidPairs()
    {
        // map from user pair names [user<m>user<n>],[user<n>,user<m>] to user pair id
        humanoid_pair_ids_map_.clear();
        // vector from user pair id to user pair
        humanoid_pairs_.clear();

        const auto & humanoids = named_humanoid_cache_.getStorage()->message_array_;

        for( auto humanoid1 = humanoids.begin(); humanoid1 != humanoids.end(); ++humanoid1 )
        {
            for( auto humanoid2 = humanoids.begin(); humanoid2 != humanoids.end(); ++humanoid2 )
            {
                if( humanoid1->name == humanoid2->name ) continue;
                if( humanoid_pair_ids_map_.count( humanoid1->name + humanoid2->name ) || humanoid_pair_ids_map_.count( humanoid2->name + humanoid1->name ) ) continue;

                humanoid_pair_ids_map_[humanoid1->name + humanoid2->name] = humanoid_pairs_.size();
                humanoid_pair_ids_map_[humanoid2->name + humanoid1->name] = humanoid_pairs_.size();
                humanoid_pairs_.push_back( _HumanoidPair( *humanoid1, *humanoid2 ) );
            }
        }
    }

    //! Get a reference to the humanoid with the given name
    _Humanoid & lookupHumanoid( const std::string & name )
    {
        return named_humanoid_cache_[name];
    }

    //! Get a reference to the joint with the given name of the humanoid with the given name
    _HumanoidJointMsg & lookupJoint( const std::string & humanoid_name, const std::string & joint_name )
    {
        return lookupHumanoid( humanoid_name )[joint_name];
    }

    //! Get a const reference to the humanoid pair made up of the humanoids with the given (order-invariant) names
    const _HumanoidPair & lookupHumanoidPair( const std::string & name1, const std::string & name2 ) const
    {
        const static _HumanoidPair default_return;
        // get a reference to the pair index, if any
        const auto & pair_index = humanoid_pair_ids_map_.find( name1 + name2 );

        if( pair_index == humanoid_pair_ids_map_.end() ) return default_return;
        return humanoid_pairs_[pair_index->second];
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( humanoidStatesCB, _HumanoidStateArrayMsg )
    {
        QUICKDEV_TRY_UPDATE_CACHE( states_cache_, msg );
        QUICKDEV_TRY_LOCK_OR_WARN( states_cache_, "Dropping message [ %s ]", QUICKDEV_GET_MESSAGE_INST_NAME( msg ).c_str() );

        QUICKDEV_GET_POLICY_NS( HumanoidRecognizer )::_HumanoidStateArrayMessageCallbackPolicy::invokeCallback( msg );
    }
};

#endif // HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERPOLICY_H_
