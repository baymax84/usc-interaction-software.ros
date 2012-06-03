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

namespace action_token_types
{
    template<class __Signature>
    struct make_signature
    {
        typedef __Signature type;
    };

    template<class __Signature>
    struct from_signature
    {
        typedef __Signature _Signature;
    };

    template<class __Return, class... __Args>
    struct from_signature<__Return( __Args... )>
    {
        typedef typename make_signature<__Return( __Args... )>::type _Signature;
        typedef __Return _Return;
        typedef quickdev::SimpleContainer<__Args...> _ArgsContainer;
    };

    template<class __Signature>
    struct from_function : public from_signature<__Signature>
    {
        //
    };

    template<template<typename> class __Function, class __Signature>
    struct from_function<__Function<__Signature> > : public from_function<__Signature>
    {
        //
    };
} // action_token

class ActionTokenImpl
{
public:
    typedef __QUICKDEV_FUNCTION_TYPE<void()> _StartCallback;
    typedef __QUICKDEV_FUNCTION_TYPE<void()> _CancelCallback;
    typedef __QUICKDEV_FUNCTION_TYPE<void()> _DoneCallback;

protected:
    quickdev::types::_TimedMutex action_mutex_;
    _StartCallback start_callback_;
    _CancelCallback cancel_callback_;
    _DoneCallback done_callback_;

public:
    ActionTokenImpl()
    {
        //
    }

    void registerCancelCB( _CancelCallback const & callback )
    {
        cancel_callback_ = callback;
    }

    void registerStartCB( _StartCallback const & callback )
    {
        start_callback_ = callback;
    }

    void registerDoneCB( _DoneCallback const & callback )
    {
        done_callback_ = callback;
    }

    void start()
    {
        if( start_callback_ ) start_callback_();
    }

    void cancel()
    {
        // unblock
        action_mutex_.unlock();
        if( cancel_callback_ ) cancel_callback_();
    }

    void done()
    {
        // unblock
        action_mutex_.unlock();
        if( done_callback_ ) done_callback_();
    }

    _StartCallback makeStartSignal()
    {
        return quickdev::auto_bind( &ActionTokenImpl::start, this );
    }

    _CancelCallback makeCancelSignal()
    {
        return quickdev::auto_bind( &ActionTokenImpl::cancel, this );
    }

    _DoneCallback makeDoneSignal()
    {
        return quickdev::auto_bind( &ActionTokenImpl::done, this );
    }

    void wait( double const & timeout = 0 )
    {
        // make sure the mutex is locked once
        action_mutex_.try_lock();

        if( timeout > 0 )
        {
            // block with timeout
            quickdev::make_unique_lock( action_mutex_, quickdev::Duration( timeout ) );
        }
        else
        {
            // block indefinitely
            action_mutex_.lock();
        }
        // unlock
        action_mutex_.unlock();
    }
};

/*
namespace action_token_types
{
    template<class __StartSignature, class __CancelSignature, class __DoneSignature>
    struct from_signatures
    {
        typedef ActionTokenBase<__StartSignature, __CancelSignature, __DoneSignature> type;
    };

    template<class __StartSignature, class __CancelSignature, class __DoneSignature>
    struct from_callbacks;

    template<class __StartReturn, class... __StartArgs, class __CancelReturn, class... __CancelArgs, class __DoneReturn, class... __DoneArgs>
    struct from_callbacks<__StartReturn(__StartArgs...), __CancelReturn(__CancelArgs...), __DoneReturn(__DoneArgs...)>
    {
        typedef typename from_signatures<__StartReturn(__DoneArgs...), __CancelReturn(__CancelArgs...), __DoneReturn(__DoneArgs...)>::type type;
    };
}
*/

class ActionToken
{
public:
    typedef ActionTokenImpl _Impl;
    typedef boost::shared_ptr<_Impl> _ImplPtr;

    typedef _Impl::_StartCallback _StartCallback;
    typedef _Impl::_CancelCallback _CancelCallback;
    typedef _Impl::_DoneCallback _DoneCallback;

protected:
    _ImplPtr impl_ptr_;

public:
/*
    template<class... __Args>
    ActionToken( __Args&&... args )
    :
        impl_ptr_( boost::make_shared<_Impl>( args... ) )
    {
        //
    }
*/

    ActionToken()
    :
        impl_ptr_( boost::make_shared<_Impl>() )
    {
        //
    }

    template<class... __Args>
    void registerCancelCB( __Args&&... args )
    {
        return impl_ptr_->registerCancelCB( args... );
    }

    template<class... __Args>
    void registerStartCB( __Args&&... args )
    {
        return impl_ptr_->registerStartCB( args... );
    }

    template<class... __Args>
    void registerDoneCB( __Args&&... args )
    {
        return impl_ptr_->registerDoneCB( args... );
    }

    void start()
    {
        return impl_ptr_->start();
    }

    void cancel()
    {
        return impl_ptr_->cancel();
    }

    void done()
    {
        return impl_ptr_->done();
    }

    template<class... __Args>
    void wait( __Args&&... args )
    {
        return impl_ptr_->wait( args... );
    }

    _StartCallback makeStartSignal()
    {
        return impl_ptr_->makeStartSignal();
    }

    _DoneCallback makeDoneSignal()
    {
        return impl_ptr_->makeDoneSignal();
    }

    _CancelCallback makeCancelSignal()
    {
        return impl_ptr_->makeCancelSignal();
    }

    _ImplPtr getImpl()
    {
        return impl_ptr_;
    }

    _Impl * get() const
    {
        return impl_ptr_.get();
    }
};

} // quickdev

/*
ActionToken sendGoal( ... )
{
    // send goal

    // make token
    ActionToken token;
    registerDoneCB( token.makeDoneCB() );
    token.registerCancelCB( quickdev::auto_bind( &ActionClientPolicy::interruptAction, this ) );
}
*/

#endif // QUICKDEVCPP_QUICKDEV_ACTIONTOKEN_H_
