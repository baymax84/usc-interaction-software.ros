/***************************************************************************
 *  include/quickdev/multi_publisher.h
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

#ifndef QUICKDEVCPP_QUICKDEV_MULTIPUBLISHER_H_
#define QUICKDEVCPP_QUICKDEV_MULTIPUBLISHER_H_

#include <quickdev/console.h>
#include <quickdev/container.h>
#include <quickdev/macros.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <map>
#include <vector>
#include <typeinfo>

//## Adapters ##########################################################

namespace ros
{

// ## PublisherAdapterStorage ##########################################
template<class __Publisher>
struct PublisherAdapterStorage {};

// ## PublisherAdapterStorage for ros::Publisher #######################
template<>
struct PublisherAdapterStorage<ros::Publisher>
{
    unsigned int cache_size_;

    PublisherAdapterStorage( const decltype( cache_size_ ) & cache_size = 10 )
    :
        cache_size_( cache_size )
    {}
};

// ## PublisherAdapter #################################################
template<class __Publisher, class __Message>
class PublisherAdapter {};

// ## PublisherAdapter for ros::Publisher ##############################
template<class __Message>
class PublisherAdapter<ros::Publisher, __Message>
{
public:
    typedef ros::Publisher _Publisher;

    static _Publisher createPublisher(
        ros::NodeHandle & nh,
        const std::string & topic,
        PublisherAdapterStorage<_Publisher> & storage )
    {
        return nh.advertise<__Message>(
            topic,
            storage.cache_size_ );
    }
};

// ## MultiPublisher ###################################################
// *given a list of message types, create a publisher and topic for each
// *provide easy functions for publishing and accessing data
template<class __Publisher = ros::Publisher>
class MultiPublisher
{
public:
    typedef std::string _Topic;
    typedef std::vector<_Topic> _TopicArray;
    typedef __Publisher _Publisher;
    typedef PublisherAdapterStorage<__Publisher> _PublisherAdapterStorage;
    typedef std::map<_Topic, __Publisher> _PublisherMap;

protected:
    _PublisherMap publishers_;

public:
    // default constructor does nothing
    MultiPublisher()
    {
        //
    }

    template<class... __Messages>
    MultiPublisher & addPublishers( ros::NodeHandle & nh, const std::initializer_list<_Topic> & topic_names_init, _PublisherAdapterStorage storage = _PublisherAdapterStorage() )
    {
        _TopicArray topic_names( topic_names_init.size() );
        std::copy( topic_names_init.begin(), topic_names_init.end(), topic_names.begin() );

        PRINT_INFO( "Attempting to add [ %zu ] publishers...", topic_names.size() );
        createPublishers<quickdev::SimpleContainer<__Messages...> >( nh, topic_names.begin(), topic_names.end(), storage );

        return *this;
    }

    // recursively process the message types in __MessagesSubset and
    // create a publisher for each
    template
    <
        class __MessagesSubset,
        typename std::enable_if<(__MessagesSubset::size_ > 0), int>::type = 0,
        typename std::enable_if<(!std::is_same<typename container::traits<__MessagesSubset>::_Front, void>::value), int>::type = 0
    >
    void createPublishers( ros::NodeHandle & nh, const typename _TopicArray::iterator & current_topic, const typename _TopicArray::iterator last_topic, _PublisherAdapterStorage & storage )
    {
        typedef typename container::traits<__MessagesSubset>::_Front _CurrentMessageType;

        const ros::NodeHandle topic_nh( nh, *current_topic );
        PRINT_INFO( ">>> Creating publisher [ %s ] on topic [ %s ]", QUICKDEV_GET_MESSAGE_NAME( _CurrentMessageType ).c_str(), topic_nh.getNamespace().c_str() );

        publishers_[*current_topic] = PublisherAdapter<__Publisher, _CurrentMessageType>::createPublisher( nh, *current_topic, storage );
        createPublishers<typename container::traits<__MessagesSubset>::_Tail>( nh, current_topic + 1, last_topic, storage );
    }

    // ignore void types
    template
    <
        class __MessagesSubset,
        typename std::enable_if<(__MessagesSubset::size_ > 0), int>::type = 0,
        typename std::enable_if<(std::is_same<typename container::traits<__MessagesSubset>::_Front, void>::value), int>::type = 0
    >
    void createPublishers( ros::NodeHandle & nh, const typename _TopicArray::iterator & current_topic, const typename _TopicArray::iterator last_topic, _PublisherAdapterStorage & storage )
    {
        createPublishers<typename container::traits<__MessagesSubset>::_Tail>( nh, current_topic + 1, last_topic, storage );
    }

    // bottom-level in the createPublishers recursive algorithm
    // __MessagesSubset should be Container<>; if so, all topics have
    // been registered
    template
    <
        class __MessagesSubset,
        typename std::enable_if<(__MessagesSubset::size_ == 0), int>::type = 0
    >
    void createPublishers( ros::NodeHandle & nh, const typename _TopicArray::iterator & current_topic, const typename _TopicArray::iterator last_topic, _PublisherAdapterStorage & storage )
    {
        //PRINT_INFO( "Finished creating topics." );
    }

    // check to see if a topic exists in the list of publishers
    bool exists( const _Topic & topic ) const
    {
        return publishers_.count( topic );
    }

    // indexing operator; allows read-only access of publishers_
    const __Publisher & operator[]( const _Topic & topic ) const
    {
        const auto & publisher( publishers_.find( topic ) );
        if( publisher != publishers_.end() ) return publisher->second;
        return __Publisher();
    }

    // indexing operator
    __Publisher & operator[]( const _Topic & topic )
    {
        static __Publisher default_return = __Publisher();

        auto publisher( publishers_.find( topic ) );
        if( publisher != publishers_.end() ) return publisher->second;
        return default_return;
    }

    // given a list of key-value pairs, recurse through the list to
    // publish the messages
    template<class __Message, class... __Args>
    void publish( const _Topic & topic, const __Message & msg, const __Args&&... args ) const
    {
        const auto & publisher( publishers_.find( topic ) );
        if( publisher != publishers_.end() ) publisher->second.publish( msg );
        else PRINT_WARN( "Cannot publish to topic [ %s ]; topic is not managed by this multi-publisher", topic.c_str() );
        publish( args... );
    }

    // bottom-level of recursive publish() algorithm
    // does nothing
    void publish() const {}
};

}

#endif // QUICKDEVCPP_QUICKDEV_MULTIPUBLISHER_H_
