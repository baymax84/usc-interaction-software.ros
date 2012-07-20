/***************************************************************************
 *  include/quickdev/action_token.h
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
 *  * Neither the name of seabee3-ros-pkg nor the names of its
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

#ifndef QUICKDEVCPP_QUICKDEV_ACTIONTOKEN_H_
#define QUICKDEVCPP_QUICKDEV_ACTIONTOKEN_H_

#include <mutex>
#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#include <quickdev/type_utils.h>
#include <quickdev/auto_bind.h>
#include <quickdev/time.h>
#include <condition_variable>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

template<class __Data>
class ActionTokenStorage
{
public:
    typedef __Data _Data;
    typedef std::vector<std::condition_variable *> _ConditionVector;

    __Data * data_ptr_;
    boost::shared_ptr<__Data> data_storage_ptr_;
    bool running_;
    bool success_;
    std::condition_variable wait_condition_;
    std::vector<std::condition_variable *> child_conditions_;
    std::mutex wait_mutex_;

    ActionTokenStorage( __Data * data_ptr = NULL )
    :
        data_ptr_( data_ptr ),
        running_( false ),
        success_( false )
    {
        //
    }

    ActionTokenStorage( std::vector<std::condition_variable *> child_conditions )
    :
        data_ptr_( NULL ),
        running_( false ),
        success_( false ),
        child_conditions_( child_conditions )
    {
        //
    }
};

template<class __Caller>
class ActionTokenBase
{
public:
    typedef __Caller _Caller;
    typedef ActionTokenStorage<__Caller> _Storage;

protected:
    boost::shared_ptr<_Storage> storage_ptr_;

public:
    ActionTokenBase()
    :
        storage_ptr_( boost::make_shared<_Storage>() )
    {
        //
    }

    ActionTokenBase( ActionTokenBase const & other )
    :
        storage_ptr_( other.storage_ptr_ )
    {
        //
    }

    ActionTokenBase( std::vector<std::condition_variable *> child_conditions )
    :
        storage_ptr_( boost::make_shared<_Storage>( child_conditions ) )
    {
        //
    }

    ActionTokenBase( __Caller * caller_ptr )
    :
        storage_ptr_( boost::make_shared<_Storage>( caller_ptr ) )
    {
        //
    }

    template
    <
        class... __Args,
        typename std::enable_if<(sizeof...(__Args) > 0), int>::type = 0
    >
    void create( __Args&&... args )
    {
        storage_ptr_->data_storage_ptr_ = boost::make_shared<__Caller>( args... );
        storage_ptr_->data_ptr_ = storage_ptr_->data_storage_ptr_.get();
    }

    boost::shared_ptr<_Storage> getStorage()
    {
        return storage_ptr_;
    }

    boost::shared_ptr<_Storage> const getStorage() const
    {
        return storage_ptr_;
    }

    bool & success()
    {
        return storage_ptr_->success_;
    }

    bool success() const
    {
        return storage_ptr_->success_;
    }

    bool & running()
    {
        return storage_ptr_->running_;
    }

    bool running() const
    {
        return storage_ptr_->running_;
    }
};

template<class __Caller>
class ActionToken : public ActionTokenBase<__Caller>
{
public:
    typedef ActionTokenBase<__Caller> _ActionTokenBase;

    template<class... __Args>
    ActionToken( __Args&&... args )
    :
        _ActionTokenBase( args... )
    {
        //
    }

    void start()
    {
        //
    }

    void cancel()
    {
        //
    }

    void wait()
    {
        //
    }
};

template<>
class ActionToken<boost::thread> : public ActionTokenBase<boost::thread>
{
public:
    typedef ActionTokenBase<boost::thread> _ActionTokenBase;

    ActionToken( ActionToken<boost::thread> const & other )
    :
        _ActionTokenBase( other )
    {
        //
    }

    template<class... __Args>
    ActionToken( __Args&&... args )
    :
        _ActionTokenBase( args... )
    {
        //
    }

    template<class... __Args>
    void start( __Args&&... args )
    {
        getStorage()->running_ = true;
        getStorage()->success_ = false;
        this->create( args... );
    }

    void cancel()
    {
        if( !ok() ) return;
        PRINT_INFO( "Got cancel signal; aborting token." );

        getStorage()->success_ = false;
        getStorage()->running_ = false;
        getStorage()->wait_condition_.notify_all();

        // unblock all child conditions
        auto & child_conditions = getStorage()->child_conditions_;
        for( auto child_condition_it = child_conditions.begin(); child_condition_it != child_conditions.end(); ++child_condition_it )
        {
            auto & child_condition = *child_condition_it;
            child_condition->notify_all();
        }
    }

    bool ok() const
    {
        return getStorage()->running_;
    }

    bool wait( double const & timeout = 0 )
    {
        auto lock = quickdev::make_unique_lock( getStorage()->wait_mutex_ );
        if( timeout > 0 )
        {
            PRINT_INFO( "Waiting for %f seconds for action to complete", timeout );
            getStorage()->wait_condition_.wait_for( lock, quickdev::Duration( timeout ) );
        }
        else
        {
            PRINT_INFO( "Waiting indefinitely for action to complete" );
            getStorage()->wait_condition_.wait( lock );
        }

        PRINT_INFO( "Token completed with state %u", getStorage()->success_ );

        return getStorage()->success_;
    }

    void complete( bool const & success = false )
    {
        PRINT_INFO( "Token completed with state %u", success );
        getStorage()->running_ = false;
        getStorage()->success_ = success;
        getStorage()->wait_condition_.notify_all();
    }
};

typedef ActionToken<boost::thread> SimpleActionToken;

namespace action_token
{
    template
    <
        class __Caller,
        class... __Args,
        typename std::enable_if<(sizeof...(__Args) != 1 || ( !std::is_same<typename variadic::element<0, __Args...>::type, __Caller &>::value && !std::is_same<typename variadic::element<0, __Args...>::type, __Caller *>::value ) ), int>::type = 0
    >
    ActionToken<__Caller> make_token( __Args&&... args )
    {
        ActionToken<__Caller> result;
        result.create( args... );
        return result;
    }

    template<class __Caller>
    ActionToken<__Caller> make_token( __Caller * caller )
    {
        return ActionToken<__Caller>( caller );
    }

    template<class __Caller>
    ActionToken<__Caller> make_token( __Caller & caller )
    {
        return ActionToken<__Caller>( &caller );
    }
} // action_token

/*
class ActionClientPolicy
{
    ActionToken<ActionClientPolicy> sendGoal( ... )
    {
        return makeToken();
    }

    ActionToken<ActionClientPolicy> makeToken()
    {
        return action_token::make_token( this );
    }

    void unregister( ActionToken<ActionClientPolicy> * token )
    {
        //
    }
};*/

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_ACTIONTOKEN_H_
