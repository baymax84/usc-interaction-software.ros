/***************************************************************************
 *  include/quickdev/multi_subscriber.h
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

#ifndef QUICKDEVCPP_QUICKDEV_MULTISUBSCRIBER_H_
#define QUICKDEVCPP_QUICKDEV_MULTISUBSCRIBER_H_

#include <quickdev/type_utils.h>
#include <quickdev/container.h>
#include <quickdev/auto_bind.h>
#include <quickdev/console.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <vector>
#include <map>

namespace ros
{

// ## SubscriberAdapterStorage #########################################
template<class __Subscriber>
struct SubscriberAdapterStorage {};

// ## SubscriberAdapterStorage for ros::Subscriber #####################
template<>
struct SubscriberAdapterStorage<ros::Subscriber>
{
    typedef SubscriberAdapterStorage<ros::Subscriber> _SubscriberAdapterStorage;
    typedef boost::shared_ptr<_SubscriberAdapterStorage> _Ptr;

    unsigned int cache_size_;

    SubscriberAdapterStorage( const decltype( cache_size_ ) & cache_size = 10 )
    :
        cache_size_( cache_size )
    {}
};

// ## SubscriberAdapter ################################################
template<class __Subscriber>
class SubscriberAdapter {};

// ## SubscriberAdapter for ros::Subscriber ############################
template<>
class SubscriberAdapter<ros::Subscriber>
{
public:
    typedef ros::Subscriber _Subscriber;
    typedef SubscriberAdapterStorage<_Subscriber> _Storage;
    typedef _Storage::_Ptr _StoragePtr;

    _StoragePtr storage_;
    _Subscriber subscriber_;

    SubscriberAdapter( _StoragePtr storage = _StoragePtr() )
    :
        storage_( storage )
    {
        //
    }

    template<class __Message>
    _Subscriber & createSubscriber(
        ros::NodeHandle & nh,
        std::string const & topic,
        std::function<void( boost::shared_ptr<__Message const> const & )> const & callback )
    {
        if( !storage_ ) storage_ = quickdev::make_shared( new _Storage() );
        subscriber_ = nh.subscribe(
            topic,
            storage_->cache_size_,
            boost::function< void( boost::shared_ptr< __Message const> const & )>( callback ) );

        return subscriber_;
    }
};


// ## MultiSubscriber ##################################################
// *given a list of message types, create a subscriber and topic for each
// *provide easy functions for publishing and accessing data
template<class __Subscriber = ros::Subscriber>
class MultiSubscriber
{
public:
    typedef std::string _Topic;
    typedef std::vector<_Topic> _TopicArray;
    typedef __Subscriber _Subscriber;
    //typedef SubscriberAdapter<__Subscriber> _SubscriberAdapter;
    typedef SubscriberAdapter<__Subscriber> _SubscriberAdapter;
    //typedef typename _SubscriberAdapter::_Storage _SubscriberAdapterStorage;
    typedef typename _SubscriberAdapter::_StoragePtr _SubscriberAdapterStoragePtr;
    typedef std::map<_Topic, _SubscriberAdapter> _SubscriberMap;

protected:
    _SubscriberMap subscribers_;

public:
    // default constructor does nothing
    MultiSubscriber()
    {
        //
    }

    // here, we only support adding one callback at a time, either through a standard or member function pointer
    template<class __Message, class __CallerBase, class __Caller>
    MultiSubscriber & addSubscriber( ros::NodeHandle & nh, _Topic const & topic_name, void( __CallerBase::*function_ptr )( __Message const & ), __Caller * const caller, _SubscriberAdapterStoragePtr storage = _SubscriberAdapterStoragePtr() )
    {
        return addSubscriber( nh, topic_name, quickdev::auto_bind( function_ptr, caller ), storage );
    }

    template<class __Message>
    MultiSubscriber & addSubscriber( ros::NodeHandle & nh, _Topic const & topic_name, __QUICKDEV_FUNCTION_TYPE< void(boost::shared_ptr< __Message const > const &)> const & callback, _SubscriberAdapterStoragePtr storage )
    {
        const ros::NodeHandle topic_nh( nh, topic_name );
        PRINT_INFO( ">>> Creating subscriber [ %s ] on topic [ %s ]", QUICKDEV_GET_MESSAGE_NAME( __Message ).c_str(), topic_nh.getNamespace().c_str() );

        SubscriberAdapter<__Subscriber> new_subscriber_adapter( storage );
        new_subscriber_adapter.createSubscriber( nh, topic_name, callback );

        subscribers_[topic_name] = new_subscriber_adapter;

        return *this;
    }

    // check to see if a topic exists in the list of subscribers
    const bool & exists( _Topic const & topic ) const
    {
        return subscribers_.count( topic );
    }

    // indexing operator; allows read-only access of subscribers_
    const __Subscriber & operator[]( _Topic const & topic ) const
    {
        const auto & subscriber( subscribers_.find( topic ) );
        if( subscriber != subscribers_.end() ) return subscriber->second.subscriber_;
        return __Subscriber();
    }
};

}

#endif // QUICKDEVCPP_QUICKDEV_MULTISUBSCRIBER_H_
