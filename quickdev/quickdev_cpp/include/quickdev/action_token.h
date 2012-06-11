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

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

template<class __Caller>
class ActionTokenBase
{
public:
    typedef __Caller _Caller;
    typedef __Caller * _CallerPtr;

protected:
    _CallerPtr caller_ptr_;
    boost::shared_ptr<__Caller> caller_storage_ptr_;

    ActionTokenBase( __Caller * caller_ptr = NULL )
    :
        caller_ptr_( caller_ptr )
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
        caller_storage_ptr_ = boost::make_shared<__Caller>( args... );
        caller_ptr_ = caller_storage_ptr_.get();
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
        this->create( args... );
    }

    void cancel()
    {
        //
    }

    void wait()
    {
        this->caller_ptr_->join();
    }
};

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
