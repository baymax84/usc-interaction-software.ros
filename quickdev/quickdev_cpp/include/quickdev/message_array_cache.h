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

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

class StampedMessageBase{};

template<class __Message>
class StampedMessage : public __Message, public StampedMessageBase
{
public:
    typedef __Message _Parent;
    struct Header
    {
        ros::Time stamp;

        Header( const ros::Time & stamp_ = ros::Time( 0 ) ) : stamp( stamp_ ){}
    };

    Header header;

    StampedMessage( const __Message & msg = __Message(), const ros::Time & stamp_ = ros::Time( 0 ) )
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

template<class __Message>
struct has_stamped_wrapper
{
    const static bool value = boost::is_base_of<StampedMessageBase, __Message>::value;
};

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
    const static bool has_stamped_wrapper_ = has_stamped_wrapper<__Message>::value;

    typedef MessageArrayCacheStorageTypesHelper<__Message, has_stamped_wrapper<__Message>::value> _MessageArrayCacheStorageTypesHelper;

    typedef typename _MessageArrayCacheStorageTypesHelper::_ROSMessage _ROSMessage;
    typedef typename _MessageArrayCacheStorageTypesHelper::_WrapperMessage _WrapperMessage;
};

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
    MessageArrayCacheStorage( const _ROSMessageArray & ros_message_array = _ROSMessageArray() )
    :
        // ros_message_array necessarily contains everything we currently know at this point
        ros_message_array_( ros_message_array ),
        // given the above, we don't need to update message_array_ following updateMessages() just this once
        needs_update_( false )
    {
        //
    }

    void registerNotifyEraseFunc( const _NotifyEraseFunc & notify_func )
    {
        notify_erase_funcs_.push_back( notify_func );
    }

    void registerNotifyUpdateFunc( const _NotifyUpdateFunc & notify_func )
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

    void push_back( const _WrapperMessage & value )
    {
        // notify all, then update
        for( auto notify_func = notify_update_funcs_.begin(); notify_func != notify_update_funcs_.end(); ++notify_func )
        {
            (*notify_func)( value );
        }
        message_array_.push_back( value );
        invalidate();
    }

    void erase( const typename _MessageArray::iterator & item )
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
    MessageArrayCache( typename _Storage::_Ptr & child_storage )
    {
        //if( child.getStorage() ) this->getStorage() = child.storage_;
        //else this->storage_ = typename _Storage::_Ptr( new _Storage() );

        if( child_storage ) this->storage_ = child_storage;
        else this->storage_ = typename _Storage::_Ptr( new _Storage() );
    }

    template<class... __StorageArgs>
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

    void updateMessage( const __Message & msg )
    {
        storage_->push_back( msg );
    }

    void updateMessages( const _ROSMessageArray & msg_array )
    {
        for( auto message = msg_array.begin(); message != msg_array.end(); ++message )
        {
            updateMessage( *message );
        }
    }
};

// consider making a flag that would override stored timestamps with the time that the message was received
struct TimedMessageArrayCacheFlags
{
    typedef unsigned int __Flag;
    const static __Flag STAMPED_ON_MESSAGES = 0;
    const static __Flag STAMPED_ON_UPDATE = 1;
};

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

    virtual void notifyUpdate( const _Message & msg ) = 0;
    virtual void notifyErase( const typename _MessageArray::iterator & msg ) = 0;

    //! called by our storage object when an item is about to be updated
    void notifyUpdate_0( const __Message & msg )
    {
        notifyUpdate( msg );
    }

    //! called by our storage object when an item is about to be erased
    void notifyErase_0( const typename _MessageArray::iterator & msg )
    {
        notifyErase( msg );
    }

    virtual void eraseOldImpl( const ros::Duration & message_lifetime, const ros::Time & now ) = 0;

    template<class __Child, typename std::enable_if<boost::is_base_of<TimedMessageArrayCacheBase<__Message>, __Child>::value, int>::type = 0>
    void tryEraseOld( __Child & child, const ros::Duration & message_lifetime, const ros::Time & now )
    {
        child.eraseOld( message_lifetime, now );
    }

    template<class __Child, typename std::enable_if<!boost::is_base_of<TimedMessageArrayCacheBase<__Message>, __Child>::value, int>::type = 0>
    void tryEraseOld( __Child & child, const ros::Duration & message_lifetime, const ros::Time & now )
    {
        //
    }

    void eraseOld( const ros::Duration & message_lifetime, const ros::Time & now )
    {
        eraseOldImpl( message_lifetime, now );
    }

    void eraseOld( const ros::Duration & message_lifetime )
    {
        eraseOld( message_lifetime, ros::Time::now() );
    }

    void eraseOld( const double & message_lifetime = 0.5 )
    {
        eraseOld( ros::Duration( message_lifetime ) );
    }

    QUICKDEV_DECLARE_ACCESSOR2( stamp_, Stamp )
};

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
    void notifyUpdate( const __Message & msg )
    {
        if( msg.header.stamp > _Parent::stamp_ ) _Parent::stamp_ = msg.header.stamp;
    }

    //! called by our parent when an item is about to be erased
    void notifyErase( const typename _MessageArray::iterator & msg )
    {
        // if the message we're about to erase has the most recent timestamp, find the next-most-recent timestamp
        if( msg->header.stamp == _Parent::stamp_ )
        {
            const auto & message_array = this->getStorage()->message_array_;
            _Parent::stamp_ = ros::Time( 0 );
            //auto last_message = message_array.begin();
            for( auto message = message_array.begin(); message != message_array.end(); ++message )
            {
                if( message->header.stamp > _Parent::stamp_ || message == message_array.begin() ) _Parent::stamp_ = message->header.stamp;
            }
        }
    }

    void eraseOldImpl( const ros::Duration & message_lifetime, const ros::Time & now )
    {
        auto & message_array = this->getStorage()->message_array_;
        //auto last_message = message_array.begin();
        //STREAM_INFO( "Filtering through " << message_array.size() << " messages." );
        size_t i = 0;
        while( i < message_array.size() )
        {
            const auto & message = message_array[i];
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
    const static bool value = is_stamped_helper<__Message>::value;
};

template<class __Message>
struct TimedMessageArrayCacheTypes
{
    const static bool is_stamped_ = is_stamped<__Message>::value;
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
    void notifyUpdate( const __Message & msg )
    {
        const auto & now = ros::Time::now();
        timestamps_.push_back( now );
        _Parent::stamp_ = now;
    }

    //! called by our parent when an item is about to be erased
    void notifyErase( const typename _MessageArray::iterator & msg )
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

    void eraseOldImpl( const ros::Duration & message_lifetime, const ros::Time & now )
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
    void notifyErase( const typename _MessageArray::iterator & msg )
    {
        const auto & erase_index = message_index_map_.find( msg->name );
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

    __Message & at( const std::string & name )
    {
        static __Message default_return;
        auto message = find( name );

        if( message != end() ) return *message;
        else return default_return;
    }

    const __Message & at( const std::string & name ) const
    {
        static __Message default_return;
        auto message = find( name );

        if( message != end() ) return *message;
        else return default_return;
    }

    __Message & operator[]( const std::string & name )
    {
        return at( name );
    }

    const __Message & operator[]( const std::string & name ) const
    {
        return at( name );
    }

    void updateMessage( const __Message & msg )
    {
        const auto & message_index = message_index_map_.find( msg.name );

        if( message_index == message_index_map_.end() )
        {
            message_index_map_[msg.name] = _Parent::getStorage()->message_array_.size();
            _Parent::updateMessage( msg );
        }
        else
        {
            *( _Parent::getStorage()->message_array_.begin() + message_index_map_[msg.name] ) = msg;
        }
    }

    void updateMessages( const typename _Parent::_ROSMessageArray & msg_array )
    {
        for( auto message = msg_array.begin(); message != msg_array.end(); ++message )
        {
            updateMessage( *message );
        }
    }
};

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_MESSAGEARRAYCACHE_H_
