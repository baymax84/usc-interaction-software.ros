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

#include <quickdev/type_utils.h>
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
    typedef PublisherAdapterStorage<ros::Publisher> _PublisherAdapterStorage;
    typedef boost::shared_ptr<_PublisherAdapterStorage> _Ptr;

    unsigned int cache_size_;
    bool latch_;

    PublisherAdapterStorage( const decltype( cache_size_ ) & cache_size = 10, const decltype( latch_ ) & latch = true )
    :
        cache_size_( cache_size ),
        latch_( latch )
    {}
};

// ## PublisherAdapter #################################################
template<class __Publisher>
class PublisherAdapter {};

// ## PublisherAdapter for ros::Publisher ##############################
template<>
class PublisherAdapter<ros::Publisher>
{
public:
    typedef ros::Publisher _Publisher;
    typedef PublisherAdapterStorage<_Publisher> _Storage;
    typedef _Storage::_Ptr _StoragePtr;

    _StoragePtr storage_;
    _Publisher publisher_;

    PublisherAdapter( _StoragePtr storage = _StoragePtr() )
    :
        storage_( storage )
    {
        //
    }

    template<class __Message>
    _Publisher & createPublisher(
        ros::NodeHandle & nh,
        std::string const & topic )
    {
        if( !storage_ ) storage_ = quickdev::make_shared( new _Storage() );
        publisher_ = nh.advertise<__Message>(
            topic,
            storage_->cache_size_,
            storage_->latch_ );

        return publisher_;
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
    typedef PublisherAdapter<__Publisher> _PublisherAdapter;
    typedef typename _PublisherAdapter::_StoragePtr _PublisherAdapterStoragePtr;
    typedef std::map<_Topic, _PublisherAdapter> _PublisherMap;

protected:
    _PublisherMap publishers_;

public:
    // default constructor does nothing
    MultiPublisher()
    {
        //
    }

    template<class... __Messages>
    MultiPublisher & addPublishers( ros::NodeHandle & nh, std::initializer_list<_Topic> const & topic_names_init, std::initializer_list<_PublisherAdapterStoragePtr> storage_init = std::initializer_list<_PublisherAdapterStoragePtr>() )
    {
        std::vector<_PublisherAdapterStoragePtr> storage( topic_names_init.size() );
        std::copy( storage_init.begin(), storage_init.end(), storage.begin() );

        PRINT_INFO( "Attempting to add [ %zu ] publishers...", topic_names_init.size() );

        createPublishers<quickdev::SimpleContainer<__Messages...> >( nh, topic_names_init.begin(), topic_names_init.end(), storage.begin(), storage.end() );

        return *this;
    }
/*
    template<class... __Messages>
    MultiPublisher & addPublishers( ros::NodeHandle & nh, std::initializer_list<_Topic> const & topic_names_init )
    {
        return addPublishers<__Messages...>( nh, topic_names_init, {} );
    }
*/
    // recursively process the message types in __MessagesSubset and
    // create a publisher for each
    template
    <
        class __MessagesSubset,
        class __TopicIterator,
        class __StorageIterator,
        typename std::enable_if<(__MessagesSubset::size_ > 0), int>::type = 0,
        typename std::enable_if<(!std::is_same<typename container::traits<__MessagesSubset>::_Front, void>::value), int>::type = 0
    >
    void createPublishers( ros::NodeHandle & nh, __TopicIterator const & current_topic, __TopicIterator const & last_topic, __StorageIterator current_storage, __StorageIterator last_storage )
    {
        typedef typename container::traits<__MessagesSubset>::_Front _CurrentMessageType;

        const ros::NodeHandle topic_nh( nh, *current_topic );
        PRINT_INFO( ">>> Creating publisher [ %s ] on topic [ %s ]", QUICKDEV_GET_MESSAGE_NAME( _CurrentMessageType ).c_str(), topic_nh.getNamespace().c_str() );

        PublisherAdapter<__Publisher> new_publisher_adapter( *current_storage );
        new_publisher_adapter.template createPublisher<_CurrentMessageType>( nh, *current_topic );

        publishers_[*current_topic] = new_publisher_adapter;

        createPublishers<typename container::traits<__MessagesSubset>::_Tail>( nh, current_topic + 1, last_topic, current_storage + 1, last_storage );
    }

    // ignore void types
    template
    <
        class __MessagesSubset,
        class __TopicIterator,
        class __StorageIterator,
        typename std::enable_if<(__MessagesSubset::size_ > 0), int>::type = 0,
        typename std::enable_if<(std::is_same<typename container::traits<__MessagesSubset>::_Front, void>::value), int>::type = 0
    >
    void createPublishers( ros::NodeHandle & nh, __TopicIterator const & current_topic, __TopicIterator const & last_topic, __StorageIterator current_storage, __StorageIterator last_storage )
    {
        PRINT_WARN( ">>> Ignoring void type for topic [ %s ]", current_topic->c_str() );
        createPublishers<typename container::traits<__MessagesSubset>::_Tail>( nh, current_topic + 1, last_topic, current_storage + 1, last_storage );
    }

    // bottom-level in the createPublishers recursive algorithm
    // __MessagesSubset should be Container<>; if so, all topics have
    // been registered
    template
    <
        class __MessagesSubset,
        class __TopicIterator,
        class __StorageIterator,
        typename std::enable_if<(__MessagesSubset::size_ == 0), int>::type = 0
    >
    void createPublishers( ros::NodeHandle & nh, __TopicIterator const & current_topic, __TopicIterator const & last_topic, __StorageIterator current_storage, __StorageIterator last_storage )
    {
//        PRINT_INFO( "Finished creating topics." );
    }

    // check to see if a topic exists in the list of publishers
    bool exists( _Topic const & topic ) const
    {
        return publishers_.count( topic );
    }

    // indexing operator; allows read-only access of publishers_
    const __Publisher & operator[]( _Topic const & topic ) const
    {
        const auto & publisher( publishers_.find( topic ) );
        if( publisher != publishers_.end() ) return publisher->second.publisher_;
        return __Publisher();
    }

    // indexing operator
    __Publisher & operator[]( _Topic const & topic )
    {
        static __Publisher default_return = __Publisher();

        auto publisher( publishers_.find( topic ) );
        if( publisher != publishers_.end() ) return publisher->second.publisher_;
        return default_return;
    }

    // given a list of key-value pairs, recurse through the list to
    // publish the messages
    template<class __Message, class... __Args>
    void publish( _Topic const & topic, __Message const & msg, __Args&&... args ) const
    {
        const auto & publisher( publishers_.find( topic ) );
        if( publisher != publishers_.end() ) publisher->second.publisher_.publish( msg );
        else PRINT_WARN( "Cannot publish to topic [ %s ]; topic is not managed by this multi-publisher", topic.c_str() );
        publish( std::forward<__Args>( args )... );
    }

    // bottom-level of recursive publish() algorithm
    // does nothing
    void publish() const {}
};

}

#endif // QUICKDEVCPP_QUICKDEV_MULTIPUBLISHER_H_
