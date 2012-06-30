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

// policies
#include <quickdev/node_handle_policy.h>
#include <quickdev/callback_policy.h>

// objects
#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/message_array_cache.h>
#include <boost/thread.hpp>
#include <map>

// utils
#include <quickdev/threading.h>
#include <humanoid/humanoid_features.h>

#include <visualization_msgs/MarkerArray.h>

QUICKDEV_DECLARE_POLICY_NS( HumanoidRecognizer )
{
    typedef quickdev::NodeHandlePolicy _NodeHandlePolicy;
    typedef quickdev::MessageCallbackPolicy<humanoid::_HumanoidStateArrayMsg> _HumanoidStateArrayMessageCallbackPolicy;
}

QUICKDEV_DECLARE_POLICY( HumanoidRecognizer, _NodeHandlePolicy, _HumanoidStateArrayMessageCallbackPolicy )

template<class __FeatureArrayMsg = void>
QUICKDEV_DECLARE_POLICY_CLASS( HumanoidRecognizer )
{
    QUICKDEV_MAKE_POLICY_FUNCS( HumanoidRecognizer )

public:
    typedef __FeatureArrayMsg _FeatureArrayMsg;
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
    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;

    // we use a ConstPtr here to simplify copying and threading
    _HumanoidStateArrayMsg::ConstPtr last_humanoid_states_msg_;

    //_HumanoidsMap humanoids_map_;
    _NamedHumanoidCache named_humanoid_cache_;
    _TimedHumanoidCache timed_humanoid_cache_;

    _HumanoidPairIdsMap humanoid_pair_ids_map_;
    _HumanoidPairs humanoid_pairs_;

    bool update_humanoids_;
    bool update_pairs_;

    std::mutex humanoids_mutex_;
    std::mutex humanoid_pairs_mutex_;
    std::mutex process_humanoids_mutex_;
    std::map<std::string, std::mutex> mutexes_;
    boost::shared_ptr<boost::thread> process_humanoids_thread_ptr_;

    bool running_;

public:
    QUICKDEV_DECLARE_ACCESSOR2( multi_pub_, MultiPub )
    QUICKDEV_DECLARE_ACCESSOR2( multi_sub_, MultiSub )

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( HumanoidRecognizer ),
        named_humanoid_cache_(),
        timed_humanoid_cache_( named_humanoid_cache_.getStorage() ),
        running_( false ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    ~HumanoidRecognizerPolicy()
    {
        running_ = false;
        humanoids_mutex_.unlock();
        humanoid_pairs_mutex_.unlock();
        process_humanoids_mutex_.unlock();
        process_humanoids_thread_ptr_->join();
    }

    bool running()
    {
        return running_ && ros::ok();
    }

    QUICKDEV_ENABLE_INIT()
    {
        QUICKDEV_GET_NODEHANDLE( nh_rel );

        update_humanoids_ = quickdev::policy::readPolicyParam( nh_rel, "update_humanoids_param", "update_humanoids", true, args... );
        update_pairs_ = quickdev::policy::readPolicyParam( nh_rel, "update_pairs_param", "update_pairs", false, args... );

        multi_pub_.addPublishers<_MarkerArrayMsg, _MarkerArrayMsg, __FeatureArrayMsg>( nh_rel, { "marker_array", "/visualization_marker_array", "features" } );
        multi_sub_.addSubscriber( nh_rel, "humanoid_states", &HumanoidRecognizerPolicy::humanoidStatesCB, this );

        process_humanoids_thread_ptr_ = boost::make_shared<boost::thread>( &HumanoidRecognizerPolicy::processHumanoids, this );

        QUICKDEV_SET_INITIALIZED();
    }

    void updateMarkers( _MarkerArrayMsg markers )
    {
        QUICKDEV_ASSERT_INITIALIZED();

        auto markers_ptr = quickdev::make_const_shared( markers );

        multi_pub_.publish( "marker_array", markers_ptr, "/visualization_marker_array", markers_ptr );
    }

    template
    <
        class __MFeatureArrayMsg,
        typename std::enable_if<(!std::is_same<__MFeatureArrayMsg, void>::value), int>::type = 0
    >
    void updateFeatures( __MFeatureArrayMsg const & features )
    {
        QUICKDEV_ASSERT_INITIALIZED();

        auto features_ptr = quickdev::make_const_shared( features );

        multi_pub_.publish( "features", features_ptr );
    }

    template<class... __ChildArgs>
    void eraseOld( __ChildArgs&&... child_args )
    {
        // remove old frames
        timed_humanoid_cache_.eraseOld( child_args... );
    }

    auto getHumanoids() -> decltype( named_humanoid_cache_ )
    {
        auto lock = quickdev::make_unique_lock( humanoids_mutex_ );
        return named_humanoid_cache_;
    }

    auto getHumanoidPairs() -> decltype( humanoid_pairs_ )
    {
        auto lock = quickdev::make_unique_lock( humanoids_mutex_ );
        return humanoid_pairs_;
    }

    //! Build a list of unique humanoids (mapped from name to _Humanoid)
    QUICKDEV_DECLARE_MESSAGE_CALLBACK( updateHumanoids, _HumanoidStateArrayMsg )
    {
        // make sure the given message isn't null
        if( !msg ) return;

        auto lock = quickdev::make_unique_lock( humanoids_mutex_ );

        for( auto humanoid_msg = msg->states.begin(); humanoid_msg != msg->states.end(); ++humanoid_msg )
        {
            named_humanoid_cache_.updateMessage( _Humanoid( *humanoid_msg ) );
        }
    }

    //! Build a list of unique humanoid pairs (mapped from combined name to _HumanoidPair)
    /*! \note buildStatesMap needs to be called first; calling buildStatesMap invalidates all data generated by this function */
    void updateHumanoidPairs()
    {
        auto humanoids_lock = quickdev::make_unique_lock( humanoids_mutex_ );
        auto humanoid_pairs_lock = quickdev::make_unique_lock( humanoid_pairs_mutex_ );

        if( named_humanoid_cache_.size() < 2 ) return;

        // map from user pair names [user<m>user<n>],[user<n>,user<m>] to user pair id
        humanoid_pair_ids_map_.clear();
        // vector from user pair id to user pair
        humanoid_pairs_.clear();

        auto const & humanoids = named_humanoid_cache_.getStorage()->message_array_;

        for( auto humanoid1 = humanoids.cbegin(); humanoid1 != humanoids.cend(); ++humanoid1 )
        {
            for( auto humanoid2 = humanoids.cbegin(); humanoid2 != humanoids.cend(); ++humanoid2 )
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
    _Humanoid & lookupHumanoid( std::string const & name )
    {
        auto lock = quickdev::make_unique_lock( humanoids_mutex_ );
        return named_humanoid_cache_[name];
    }

    //! Get a reference to the joint with the given name of the humanoid with the given name
    _HumanoidJointMsg & lookupJoint( std::string const & humanoid_name, std::string const & joint_name )
    {
        auto lock = quickdev::make_unique_lock( humanoids_mutex_ );
        return lookupHumanoid( humanoid_name )[joint_name];
    }

    //! Get a const reference to the humanoid pair made up of the humanoids with the given (order-invariant) names
    const _HumanoidPair & lookupHumanoidPair( std::string const & name1, std::string const & name2 ) const
    {
        const static _HumanoidPair default_return;
        // get a reference to the pair index, if any
        const auto & pair_index = humanoid_pair_ids_map_.find( name1 + name2 );

        if( pair_index == humanoid_pair_ids_map_.end() ) return default_return;
        return humanoid_pairs_[pair_index->second];
    }

    // perform computation in a separate thread to prevent blocking in callbacks
    void processHumanoids()
    {
        while( running() )
        {
            process_humanoids_mutex_.try_lock();
            auto process_humanoids_lock = quickdev::make_unique_lock( process_humanoids_mutex_ );

            if( !last_humanoid_states_msg_ ) continue;

            if( update_humanoids_ ) updateHumanoids( last_humanoid_states_msg_ );
            if( update_pairs_ ) updateHumanoidPairs();

            QUICKDEV_GET_POLICY_NS( HumanoidRecognizer )::_HumanoidStateArrayMessageCallbackPolicy::invokeCallback( last_humanoid_states_msg_ );
        }
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( humanoidStatesCB, _HumanoidStateArrayMsg )
    {
        last_humanoid_states_msg_ = msg;

        // wake up the processHumanoids thread for one cycle
        process_humanoids_mutex_.unlock();
    }
};

#endif // HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERPOLICY_H_
