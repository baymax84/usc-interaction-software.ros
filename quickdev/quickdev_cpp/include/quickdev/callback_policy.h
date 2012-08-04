/***************************************************************************
 *  include/quickdev/callback_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_CALLBACKPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_CALLBACKPOLICY_H_

#include <quickdev/policy.h>
#include <deque>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

// =============================================================================================================================================
//! Generic callback policy for any kind of return type/arg types
QUICKDEV_DECLARE_POLICY( Callback, Policy );

template<class __Signature> class CallbackPolicy;

template<class __CallbackReturn, class... __CallbackArgs>
class CallbackPolicy<__CallbackReturn( __CallbackArgs... )> : public QUICKDEV_GET_POLICY_ADAPTER_WITH_NS( Callback )
{
    QUICKDEV_MAKE_POLICY_FUNCS( Callback )

public:
    typedef __QUICKDEV_FUNCTION_TYPE<__CallbackReturn( __CallbackArgs&&... )> _CallbackType;

private:
    std::deque<_CallbackType> callbacks_;

    // =========================================================================================================================================
    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( Callback )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    // =========================================================================================================================================
    void registerCallback( _CallbackType const & callback );

    template<class __Return>
    QUICKDEV_ENABLE_IF_SAME( __CallbackReturn, __Return, void )
    invokeCallback_0( __CallbackArgs&&... args ) const
    {
        for( auto callback_it = callbacks_.begin(); callback_it != callbacks_.end(); ++callback_it )
        {
            if( *callback_it ) (*callback_it)( std::forward<__CallbackArgs>( args )... );
        }
    }

    template<class __Return>
    QUICKDEV_ENABLE_IF_NOT_SAME( __CallbackReturn, __Return, void )
    invokeCallback_0( __CallbackArgs&&... args ) const
    {
        __CallbackReturn default_return = __CallbackReturn();

        for( auto callback_it = callbacks_.begin(); callback_it != callbacks_.end(); ++callback_it )
        {
            if( *callback_it ) default_return = (*callback_it)( std::forward<__CallbackArgs>( args )... );
        }

        return default_return;
    }

    __CallbackReturn invokeCallback( __CallbackArgs&&... args ) const
    {
        // in order to enable/disable functions with enable_if, they need to be directly dependent on some outer type
        return invokeCallback_0<__CallbackReturn>( std::forward<__CallbackArgs>( args )... );
    }
};

// #############################################################################################################################################

// =============================================================================================================================================
//! Type Adapters for CallbackPolicy
QUICKDEV_DECLARE_POLICY_NAMESPACE( Callback )
{

template<class __Signature> struct from_function;

template<class __Return, class... __Args>
struct from_function<__QUICKDEV_FUNCTION_TYPE<__Return( __Args... )> >
{
    typedef CallbackPolicy<__Return( __Args... )> type;
};

} // QUICKDEV_DECLARE_POLICY_NAMESPACE( Callback )

// #############################################################################################################################################

// =============================================================================================================================================
//! Special callback policy for message-based callbacks
template<class __Message>
class MessageCallbackPolicy : public CallbackPolicy<void( typename __Message::ConstPtr const & )>
{
    QUICKDEV_MAKE_POLICY_FUNCS( MessageCallback )

public:
    typedef CallbackPolicy<void( typename __Message::ConstPtr const & )> _CallbackPolicy;

    // =========================================================================================================================================
    template<class... __Args>
    MessageCallbackPolicy( __Args&&... args ) : _CallbackPolicy( std::forward<__Args>( args )... )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }
};

// #############################################################################################################################################

// =============================================================================================================================================
//! Special callback policy for service-based callbacks
template<class __Service>
class ServiceCallbackPolicy : public CallbackPolicy<bool( typename __Service::Request &, typename __Service::Response & )>
{
    QUICKDEV_MAKE_POLICY_FUNCS( ServiceCallback )

public:
    typedef CallbackPolicy<bool( typename __Service::Request &, typename __Service::Response & )> _CallbackPolicy;

    // =========================================================================================================================================
    template<class... __Args>
    ServiceCallbackPolicy( __Args&&... args ) : _CallbackPolicy( std::forward<__Args>( args )... )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }
};

// #############################################################################################################################################

#include <quickdev/details/callback_policy_impl.h>

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_CALLBACKPOLICY_H_
