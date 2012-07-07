/***************************************************************************
 *  include/quickdev/message_array_cache.h
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

#ifndef QUICKDEVCPP_QUICKDEV_MESSAGEARRAYCACHE_H_
#define QUICKDEVCPP_QUICKDEV_MESSAGEARRAYCACHE_H_

#include <quickdev/auto_bind.h>
#include <quickdev/console.h>

#include <ros/time.h>

#include <type_traits>
#include <map>
#include <vector>
#include <deque>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <quickdev/type_utils.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{


// #############################################################################################################################################
// Stamped Message Wrapper
class StampedMessageBase{};

template<class __Message>
class StampedMessage : public __Message, public StampedMessageBase
{
public:
    typedef __Message _Parent;
    struct Header
    {
        ros::Time stamp;

        Header( ros::Time const & stamp_ = ros::Time( 0 ) ) : stamp( stamp_ ){}
    };

    Header header;

    StampedMessage( __Message const & msg = __Message(), ros::Time const & stamp_ = ros::Time( 0 ) )
    :
        _Parent( msg ), header( stamp_ )
    {
        //
    }

    __Message getParent()
    {
        return (__Message) *this;
    }
};

// #############################################################################################################################################
// Stamped Message Traits

template<class __Message>
struct has_stamped_wrapper
{
    static bool const value = boost::is_base_of<StampedMessageBase, __Message>::value;
};

// #############################################################################################################################################
// Message Array Cache Storage Traits

template<class __Message, bool has_stamped_wrapper>
struct MessageArrayCacheStorageTypesHelper{};

template<class __Message>
struct MessageArrayCacheStorageTypesHelper<__Message, true>
{
    typedef typename __Message::_Parent _ROSMessage;
    typedef __Message _WrapperMessage;
};

template<class __Message>
struct MessageArrayCacheStorageTypesHelper<__Message, false>
{
    typedef __Message _ROSMessage;
    typedef __Message _WrapperMessage;
};

template<class __Message>
struct MessageArrayCacheStorageTypes
{
    static bool const has_stamped_wrapper_ = has_stamped_wrapper<__Message>::value;

    typedef MessageArrayCacheStorageTypesHelper<__Message, has_stamped_wrapper<__Message>::value> _MessageArrayCacheStorageTypesHelper;

    typedef typename _MessageArrayCacheStorageTypesHelper::_ROSMessage _ROSMessage;
    typedef typename _MessageArrayCacheStorageTypesHelper::_WrapperMessage _WrapperMessage;
};

// #############################################################################################################################################
// Message Array Cache Storage

template<class __Message>
class MessageArrayCacheStorage
{
public:
    typedef MessageArrayCacheStorageTypes<__Message> _MessageArrayCacheStorageTypes;
    typedef typename _MessageArrayCacheStorageTypes::_ROSMessage _ROSMessage;
    typedef typename _MessageArrayCacheStorageTypes::_WrapperMessage _WrapperMessage;

    typedef std::deque<_WrapperMessage> _MessageArray;
    typedef std::vector<_ROSMessage> _ROSMessageArray;
    typedef MessageArrayCacheStorage<__Message> _MessageArrayCacheStorage;
    typedef boost::shared_ptr<_MessageArrayCacheStorage> _Ptr;
    typedef __QUICKDEV_FUNCTION_TYPE<void( const typename _MessageArray::iterator & )> _NotifyEraseFunc;
    typedef __QUICKDEV_FUNCTION_TYPE<void( const _WrapperMessage & )> _NotifyUpdateFunc;

    std::deque<_NotifyEraseFunc> notify_erase_funcs_;
    std::deque<_NotifyUpdateFunc> notify_update_funcs_;

    _MessageArray message_array_;
    _ROSMessageArray ros_message_array_;

protected:
    bool needs_update_;

public:
    MessageArrayCacheStorage( _ROSMessageArray const & ros_message_array = _ROSMessageArray() )
    :
        // ros_message_array necessarily contains everything we currently know at this point
        ros_message_array_( ros_message_array ),
        // given the above, we don't need to update message_array_ following updateMessages() just this once
        needs_update_( false )
    {
        //
    }

    void registerNotifyEraseFunc( _NotifyEraseFunc const & notify_func )
    {
        notify_erase_funcs_.push_back( notify_func );
    }

    void registerNotifyUpdateFunc( _NotifyUpdateFunc const & notify_func )
    {
        notify_update_funcs_.push_back( notify_func );
    }

    template<bool has_stamped_wrapper, typename std::enable_if<(has_stamped_wrapper), int>::type = 0>
    void copyMessages()
    {
        ros_message_array_.resize( message_array_.size() );
        auto ros_message = ros_message_array_.begin();
        auto wrapper_message = message_array_.begin();

        while( ros_message != ros_message_array_.end() )
        {
            *ros_message = wrapper_message->getParent();
            ++ros_message;
            ++wrapper_message;
        }
    }

    template<bool has_stamped_wrapper, typename std::enable_if<(!has_stamped_wrapper), int>::type = 0>
    void copyMessages()
    {
        ros_message_array_.resize( message_array_.size() );
        std::copy( message_array_.begin(), message_array_.end(), ros_message_array_.begin() );
    }

    auto getMessages() -> decltype( ros_message_array_ ) &
    {
        if( needs_update_ )
        {
            copyMessages<_MessageArrayCacheStorageTypes::has_stamped_wrapper_>();
            needs_update_ = false;
        }
        return ros_message_array_;
    }

    auto getRawMessages() const -> const decltype( message_array_ ) &
    {
        return message_array_;
    }

    void invalidate()
    {
        needs_update_ = true;
    }

    void push_back( _WrapperMessage const & value )
    {
        // notify all, then update
        for( auto notify_func = notify_update_funcs_.begin(); notify_func != notify_update_funcs_.end(); ++notify_func )
        {
            (*notify_func)( value );
        }
        message_array_.push_back( value );
        invalidate();
    }

    void erase( typename _MessageArray::iterator const & item )
    {
        if( message_array_.size() == 0 || item == message_array_.end() ) return;

        // notify all, then erase
        for( auto notify_func = notify_erase_funcs_.begin(); notify_func != notify_erase_funcs_.end(); ++notify_func )
        {
            (*notify_func)( item );
        }
        //STREAM_INFO( "Attempting to erase item: " << item->data << ". " << message_array_.size() << " items remain." );
        message_array_.erase( item );
        invalidate();
    }
};

// #############################################################################################################################################
// Message Array Cache

template<class __Message>
class MessageArrayCache
{
public:
    typedef __Message _Message;
    typedef MessageArrayCacheStorage<__Message> _Storage;
    typedef typename _Storage::_MessageArray _MessageArray;
    typedef typename _Storage::_ROSMessageArray _ROSMessageArray;
    typedef MessageArrayCache<__Message> _MessageArrayCache;

private:
    typename _Storage::_Ptr storage_;

public:
    //template<class __Child, typename std::enable_if<boost::is_base_of<_MessageArrayCache, __Child>::value, int>::type = 0>
    //MessageArrayCache( __Child & child )
    MessageArrayCache()
    {
        this->storage_ = typename _Storage::_Ptr( new _Storage() );
    }

    MessageArrayCache( typename _Storage::_Ptr & child_storage )
    {
        //if( child.getStorage() ) this->getStorage() = child.storage_;
        //else this->storage_ = typename _Storage::_Ptr( new _Storage() );

        if( child_storage ) this->storage_ = child_storage;
        else this->storage_ = typename _Storage::_Ptr( new _Storage() );
    }

    template
    <
        class __StorageArg,
        typename std::enable_if<(!boost::is_base_of<MessageArrayCache<__Message>, __StorageArg>::value && !quickdev::is_same_type<typename _Storage::_Ptr, __StorageArg>::value), int>::type = 0
    >
    MessageArrayCache( __StorageArg const & storage_arg )
    :
        storage_( new _Storage( storage_arg ) )
    {
        // we still want to build our cache from the current info
        updateMessages( storage_->getMessages() );
    }

    template
    <
        class... __StorageArgs,
        typename std::enable_if<(sizeof...(__StorageArgs) > 1), int>::type = 0
    >
    MessageArrayCache( __StorageArgs&&... storage_args )
    :
        storage_( new _Storage( storage_args... ) )
    {
        // we still want to build our cache from the current info
        updateMessages( storage_->getMessages() );
    }

    inline _MessageArrayCache & getInstance(){ return *this; }

    // read access for storage_ (via getStorage)
    QUICKDEV_DECLARE_ACCESSOR2( storage_, Storage )

    _ROSMessageArray getMessages()
    {
        return storage_->getMessages();
    }

    const _MessageArray & getRawMessages() const
    {
        return storage_->getRawMessages();
    }

    void updateMessage( __Message const & msg )
    {
        storage_->push_back( msg );
    }

    void updateMessages( _ROSMessageArray const & msg_array )
    {
        for( auto message = msg_array.begin(); message != msg_array.end(); ++message )
        {
            updateMessage( *message );
        }
    }
};

// #############################################################################################################################################
// Compile-Time Flags for Timed Message Array Cache

struct TimedMessageArrayCacheFlags
{
    typedef unsigned int __Flag;
    static __Flag const STAMPED_ON_MESSAGES = 0;
    static __Flag const STAMPED_ON_UPDATE = 1;
};

// #############################################################################################################################################
// Timed Message Array Cache Base

template<class __Message>
class TimedMessageArrayCacheBase : public MessageArrayCache<__Message>
{
public:
    typedef MessageArrayCache<__Message> _Parent;
    typedef __Message _Message;
    typedef typename _Parent::_MessageArray _MessageArray;
    typedef typename _Parent::_ROSMessageArray _ROSMessageArray;

protected:
    ros::Time stamp_;

public:
    template<class... __ParentArgs>
    TimedMessageArrayCacheBase( __ParentArgs&&... parent_args )
    :
        _Parent( parent_args... ), stamp_( 0 )
    {
        this->getStorage()->registerNotifyEraseFunc( quickdev::auto_bind( &TimedMessageArrayCacheBase::notifyErase_0, this ) );
        this->getStorage()->registerNotifyUpdateFunc( quickdev::auto_bind( &TimedMessageArrayCacheBase::notifyUpdate_0, this ) );
        // we still want to build our cache from the current info
        //eraseOld();
    }

    virtual void notifyUpdate( _Message const & msg ) = 0;
    virtual void notifyErase( typename _MessageArray::iterator const & msg ) = 0;

    //! called by our storage object when an item is about to be updated
    void notifyUpdate_0( __Message const & msg )
    {
        notifyUpdate( msg );
    }

    //! called by our storage object when an item is about to be erased
    void notifyErase_0( typename _MessageArray::iterator const & msg )
    {
        notifyErase( msg );
    }

    virtual void eraseOldImpl( ros::Duration const & message_lifetime, ros::Time const & now ) = 0;

    template<class __Child, typename std::enable_if<boost::is_base_of<TimedMessageArrayCacheBase<__Message>, __Child>::value, int>::type = 0>
    void tryEraseOld( __Child & child, ros::Duration const & message_lifetime, ros::Time const & now )
    {
        child.eraseOld( message_lifetime, now );
    }

    template<class __Child, typename std::enable_if<!boost::is_base_of<TimedMessageArrayCacheBase<__Message>, __Child>::value, int>::type = 0>
    void tryEraseOld( __Child & child, ros::Duration const & message_lifetime, ros::Time const & now )
    {
        //
    }

    void eraseOld( ros::Duration const & message_lifetime, ros::Time const & now )
    {
        eraseOldImpl( message_lifetime, now );
    }

    void eraseOld( ros::Duration const & message_lifetime )
    {
        eraseOld( message_lifetime, ros::Time::now() );
    }

    void eraseOld( double const & message_lifetime = 0.5 )
    {
        eraseOld( ros::Duration( message_lifetime ) );
    }

    QUICKDEV_DECLARE_ACCESSOR2( stamp_, Stamp )
};

// #############################################################################################################################################
// Timed Message Array Cache Compile-Time Flags Adapter

template<class __Message, unsigned int __Flag__>
class TimedMessageArrayCacheHelper{};

template<class __Message>
class TimedMessageArrayCacheHelper<__Message, TimedMessageArrayCacheFlags::STAMPED_ON_MESSAGES> : public TimedMessageArrayCacheBase<__Message>
{
public:
    typedef TimedMessageArrayCacheBase<__Message> _Parent;
    typedef __Message _Message;
    typedef typename _Parent::_MessageArray _MessageArray;
    typedef typename _Parent::_ROSMessageArray _ROSMessageArray;

public:
    template<class... __ParentArgs>
    TimedMessageArrayCacheHelper( __ParentArgs&&... parent_args )
    :
        _Parent( parent_args... )
    {
        //
    }

    //! called by our parent when an item is about to be updated
    void notifyUpdate( __Message const & msg )
    {
        if( msg.header.stamp > _Parent::stamp_ ) _Parent::stamp_ = msg.header.stamp;
    }

    //! called by our parent when an item is about to be erased
    void notifyErase( typename _MessageArray::iterator const & msg )
    {
        // if the message we're about to erase has the most recent timestamp, find the next-most-recent timestamp
        if( msg->header.stamp == _Parent::stamp_ )
        {
            auto const & message_array = this->getStorage()->message_array_;
            _Parent::stamp_ = ros::Time( 0 );
            //auto last_message = message_array.begin();
            for( auto message = message_array.begin(); message != message_array.end(); ++message )
            {
                if( message->header.stamp > _Parent::stamp_ || message == message_array.begin() ) _Parent::stamp_ = message->header.stamp;
            }
        }
    }

    void eraseOldImpl( ros::Duration const & message_lifetime, ros::Time const & now )
    {
        auto & message_array = this->getStorage()->message_array_;
        //auto last_message = message_array.begin();
        //STREAM_INFO( "Filtering through " << message_array.size() << " messages." );
        size_t i = 0;
        while( i < message_array.size() )
        {
            auto const & message = message_array[i];
            if( now - message.header.stamp > message_lifetime )
            {
                _Parent::getStorage()->erase( message_array.begin() + i );
            }
            else
            {
                _Parent::tryEraseOld( message, message_lifetime, now );
                ++i;
            }
        }
    }
};

/*template<class __Message>
struct is_stamped_helper{};

struct is_stamped_helper
{
    //
};
template<class __Message>
struct is_stamped_helper
{
    //
};

template<class __Message>
struct is_stamped
{
    static bool const value = is_stamped_helper<__Message>::value;
};

template<class __Message>
struct TimedMessageArrayCacheTypes
{
    static bool const is_stamped_ = is_stamped<__Message>::value;
};*/

template<class __Message>
class TimedMessageArrayCacheHelper<__Message, TimedMessageArrayCacheFlags::STAMPED_ON_UPDATE> : public TimedMessageArrayCacheBase<__Message>
{
public:
    typedef TimedMessageArrayCacheBase<__Message> _Parent;
    typedef __Message _Message;
    typedef typename _Parent::_MessageArray _MessageArray;
    typedef typename _Parent::_ROSMessageArray _ROSMessageArray;

protected:
    std::deque<ros::Time> timestamps_;

public:
    template<class... __ParentArgs>
    TimedMessageArrayCacheHelper( __ParentArgs&&... parent_args )
    :
        _Parent( parent_args... )
    {
        //
    }

    //! called by our parent when an item is about to be updated
    void notifyUpdate( __Message const & msg )
    {
        auto const & now = ros::Time::now();
        timestamps_.push_back( now );
        _Parent::stamp_ = now;
    }

    //! called by our parent when an item is about to be erased
    void notifyErase( typename _MessageArray::iterator const & msg )
    {
        // if the message we're about to erase has the most recent timestamp, find the most-recent timestamp
        // this will necessarily be the message closest to the back of the queue that is not the current message
        if( timestamps_.size() > 1 )
        {
            if( msg == this->getStorage()->message_array_.end() - 1 ) _Parent::stamp_ = *( timestamps_.end() - 2 );
            else _Parent::stamp_ = *( timestamps_.end() - 1 );
        }
        else _Parent::stamp_ = ros::Time( 0 );
    }

    void eraseOldImpl( ros::Duration const & message_lifetime, ros::Time const & now )
    {
        auto & message_array = this->getStorage()->message_array_;
        //auto last_message = message_array.begin();
        while( message_array.size() > 0 )
        {
            auto message = message_array.begin();
            auto timestamp = timestamps_.begin();

            if( now - *timestamp > message_lifetime )
            {
                _Parent::getStorage()->erase( message );
                timestamps_.erase( timestamp );
            }
            // if we find a message that is sufficiently recent, then all following messages must also be sufficiently recent
            else
            {
                _Parent::tryEraseOld( *message, message_lifetime, now );
                break;
            }
        }
    }
};

// #############################################################################################################################################
// Timed Message Array Cache

template<class __Message, unsigned int __Flag__ = TimedMessageArrayCacheFlags::STAMPED_ON_MESSAGES>
class TimedMessageArrayCache : public TimedMessageArrayCacheHelper<__Message, __Flag__>
{
public:
    typedef TimedMessageArrayCacheHelper<__Message, __Flag__> _Parent;
    typedef __Message _Message;
    typedef typename _Parent::_MessageArray _MessageArray;
    typedef typename _Parent::_ROSMessageArray _ROSMessageArray;

public:

    template<class... __ParentArgs>
    TimedMessageArrayCache( __ParentArgs&&... parent_args )
    :
        _Parent( parent_args... )
    {
        //
    }

    inline TimedMessageArrayCache<__Message> & getInstance(){ return *this; }

    size_t size() const
    {
        return this->getStorage()->message_array_.size();
    }
};

// #############################################################################################################################################
// Named Message Array Cache

template<class __Message>
class NamedMessageArrayCache : public MessageArrayCache<__Message>
{
public:
    typedef MessageArrayCache<__Message> _Parent;
    typedef __Message _Message;
    typedef typename _Parent::_MessageArray _MessageArray;
    typedef typename _Parent::_ROSMessageArray _ROSMessageArray;
    typedef std::map<std::string, size_t> _MessageIndexMap;

protected:
    _MessageIndexMap message_index_map_;

public:
    template<class... __ParentArgs>
    NamedMessageArrayCache( __ParentArgs&&... parent_args )
    :
        _Parent( parent_args... )
    {
        _Parent::getStorage()->registerNotifyEraseFunc( quickdev::auto_bind( &NamedMessageArrayCache::notifyErase, this ) );
        // we still want to build our cache from the current info
        updateMessages( _Parent::getStorage()->getMessages() );
    }

    inline NamedMessageArrayCache<__Message> & getInstance(){ return *this; }

    //! called by our storage object when an item is about to be erased
    void notifyErase( typename _MessageArray::iterator const & msg )
    {
        auto const & erase_index = message_index_map_.find( msg->name );
        // starting at the item to erase, decrement all indices by 1
        for( auto message_index = erase_index; message_index != message_index_map_.end(); ++message_index )
        {
            --message_index->second;
        }

        // erase the indicated item from our index map
        message_index_map_.erase( erase_index );
    }

    size_t size() const
    {
        return message_index_map_.size();
    }

    typename _MessageArray::iterator find( std::string name )
    {
        auto index = message_index_map_.find( name );

        if( index != message_index_map_.end() ) return _Parent::getStorage()->message_array_.begin() + index->second;
        else return _Parent::getStorage()->message_array_.end();
    }

    typename _MessageArray::const_iterator find( std::string name ) const
    {
        auto index = message_index_map_.find( name );

        if( index != message_index_map_.end() ) return _Parent::getStorage()->message_array_.begin() + index->second;
        else return _Parent::getStorage()->message_array_.end();
    }

    typename _MessageArray::iterator begin()
    {
        return _Parent::getStorage()->message_array_.begin();
    }

    typename _MessageArray::const_iterator begin() const
    {
        return cbegin();
    }

    typename _MessageArray::const_iterator cbegin() const
    {
        return _Parent::getStorage()->message_array_.cbegin();
    }

    typename _MessageArray::iterator end()
    {
        return _Parent::getStorage()->message_array_.end();
    }

    typename _MessageArray::const_iterator end() const
    {
        return cend();
    }

    typename _MessageArray::const_iterator cend() const
    {
        return _Parent::getStorage()->message_array_.cend();
    }

    __Message & at( std::string const & name )
    {
        static __Message default_return;
        auto message = find( name );

        if( message != end() ) return *message;
        else return default_return;
    }

    const __Message & at( std::string const & name ) const
    {
        static __Message default_return;
        auto message = find( name );

        if( message != end() ) return *message;
        else return default_return;
    }

    __Message & operator[]( std::string const & name )
    {
        return at( name );
    }

    const __Message & operator[]( std::string const & name ) const
    {
        return at( name );
    }

    void updateMessage( __Message const & msg )
    {
        auto message_index_it = message_index_map_.find( msg.name );

        if( message_index_it == message_index_map_.end() )
        {
            message_index_map_[msg.name] = _Parent::getStorage()->message_array_.size();
            _Parent::updateMessage( msg );
        }
        else
        {
            *( _Parent::getStorage()->message_array_.begin() + message_index_it->second ) = msg;
        }
    }

    //! function to update cache with message stored in any kind of smart pointer
    template<template<typename> class __Ptr>
    void updateMessage( __Ptr<__Message> const & msg )
    {
        updateMessage( *msg );
    }

    void updateMessages( typename _Parent::_ROSMessageArray const & msg_array )
    {
        for( auto message = msg_array.begin(); message != msg_array.end(); ++message )
        {
            updateMessage( *message );
        }
    }
};

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_MESSAGEARRAYCACHE_H_
