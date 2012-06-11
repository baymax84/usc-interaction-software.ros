/***************************************************************************
 *  include/quickdev/auto_bind.h
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

#ifndef QUICKDEVCPP_QUICKDEV_AUTOBIND_H_
#define QUICKDEVCPP_QUICKDEV_AUTOBIND_H_

#include <quickdev/macros.h>
#include <quickdev/container.h>
#include <functional>

/* auto_binder inspired by code written by "superbonzo" and "Chris_F" (http://www.codeguru.com/forum/showthread.php?t=512875) */

#define __AUTO_BIND_FUNCTION_TYPE __QUICKDEV_FUNCTION_TYPE

namespace details
{

// =============================================================================================================================================
template<class __Return, class __Container>
struct make_function{};

template<class __Return, template<typename...> class __Container, class... __Args>
struct make_function<__Return, __Container<__Args...> >
{
    typedef __AUTO_BIND_FUNCTION_TYPE<__Return(__Args...)> function_type;
};

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

// #############################################################################################################################################
template<int N>
struct auto_binder
{
    // =========================================================================================================================================
    template
    <
        class __CallerType,
        class __ReturnType,
        class... __ArgTypes,
        class... __PlaceHolders
    >
    static __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> auto_bind(__ReturnType(__CallerType::*function_ptr)(__ArgTypes...), __CallerType* const caller, __PlaceHolders&&... placeholders )
    {
        return auto_binder<N-1>::auto_bind( function_ptr, caller, std::_Placeholder<N>(), std::forward<__PlaceHolders>( placeholders )... );
    }

    // =========================================================================================================================================
    template
    <
        class __ReturnType,
        class... __ArgTypes,
        class... __PlaceHolders
    >
    static __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> auto_bind(__ReturnType(*function_ptr)(__ArgTypes...), __PlaceHolders&&... placeholders )
    {
        return auto_binder<N-1>::auto_bind( function_ptr, std::_Placeholder<N>(), std::forward<__PlaceHolders>( placeholders )... );
    }

    // =========================================================================================================================================
    template
    <
        class... __OutputArgTypes,
        class __ReturnType,
        class... __InputArgTypes,
        class... __Appends
    >
    static __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__OutputArgTypes...)> auto_bind_append( const quickdev::SimpleContainer<__OutputArgTypes...> & container, const __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__InputArgTypes...)> & function, __Appends&&... appends )
    {
        return auto_binder<N-1>::auto_bind_append( container, function, std::_Placeholder<N>(), std::forward<__Appends>( appends )... );
    }
/*
    template
    <
        class... __OutputArgTypes,
        class __ReturnType,
        class... __InputArgTypes,
        class... __Prepends
    >
    static __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__OutputArgTypes...)> auto_bind_prepend( const quickdev::SimpleContainer<__OutputArgTypes...> & container, const __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__InputArgTypes...)> & function, __Prepends&&... prepends )
    {
        return auto_binder<N-1>::auto_bind_prepend( container, function, std::_Placeholder<N>(), std::forward<__Prepends>( prepends )... );
    }
*/
};

// #############################################################################################################################################
template<>
struct auto_binder<0>
{
    // =========================================================================================================================================
    template
    <
        class __CallerType,
        class __ReturnType,
        class... __ArgTypes,
        class... __PlaceHolders
    >
    static __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> auto_bind(__ReturnType(__CallerType::*function_ptr)(__ArgTypes...), __CallerType* const caller, __PlaceHolders&&... placeholders )
    {
        return __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)>( std::bind( function_ptr, caller, std::forward<__PlaceHolders>( placeholders )... ) );
    }

    // =========================================================================================================================================
    template
    <
        class __ReturnType,
        class... __ArgTypes,
        class... __PlaceHolders
    >
    static __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> auto_bind(__ReturnType(*function_ptr)(__ArgTypes...), __PlaceHolders&&... placeholders )
    {
        return __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)>( std::bind( function_ptr, std::forward<__PlaceHolders>( placeholders )... ) );
    }

    // =========================================================================================================================================
    template
    <
        class... __OutputArgTypes,
        class __ReturnType,
        class... __InputArgTypes,
        class... __Appends
    >
    static __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__OutputArgTypes...)> auto_bind_append( const quickdev::SimpleContainer<__OutputArgTypes...> & container, const __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__InputArgTypes...)> & function, __Appends&&... appends )
    {
        return __AUTO_BIND_FUNCTION_TYPE<__ReturnType( __OutputArgTypes... )>( std::bind( function, std::forward<__Appends>( appends )... ) );
    }
/*
    template
    <
        class... __OutputArgTypes,
        class __ReturnType,
        class... __InputArgTypes,
        class... __Prepends
    >
    static __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__OutputArgTypes...)> auto_bind_prepend( const quickdev::SimpleContainer<__OutputArgTypes...> & container, const __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__InputArgTypes...)> & function, __Prepends&&... prepends )
    {
        return __AUTO_BIND_FUNCTION_TYPE<__ReturnType( __OutputArgTypes... )>( std::bind( function, std::forward<__Prepends>( prepends )... ) );
    }
*/
};

} // details

// #############################################################################################################################################

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

// =============================================================================================================================================
template
<
    class __Signature,
    class __CallerType,
    class... __ArgTypes
>
__AUTO_BIND_FUNCTION_TYPE<__Signature> auto_bind( typename details::from_function<__Signature>::_Return( __CallerType::*function_ptr )( __ArgTypes... ), __CallerType* const caller )
{
    return details::auto_binder<sizeof...(__ArgTypes)>::auto_bind( function_ptr, caller );
}

// =============================================================================================================================================
template
<
    class __CallerType,
    class __ReturnType,
    class... __ArgTypes
>
__AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> auto_bind( __ReturnType( __CallerType::*function_ptr )( __ArgTypes... ), __CallerType* const caller )
{
    return details::auto_binder<sizeof...(__ArgTypes)>::auto_bind( function_ptr, caller );
}

// =============================================================================================================================================
template
<
    class __ReturnType,
    class... __ArgTypes
>
__AUTO_BIND_FUNCTION_TYPE<__ReturnType(__ArgTypes...)> auto_bind( __ReturnType( *function_ptr )( __ArgTypes... ) )
{
    return details::auto_binder<sizeof...(__ArgTypes)>::auto_bind( function_ptr );
}

// =============================================================================================================================================
template
<
    class __ReturnType,
    class... __InputArgTypes,
    class... __Appends,
    typename std::enable_if<(sizeof...(__Appends) > 0), int>::type = 0
>
typename details::make_function<__ReturnType, typename container::subtype<0, sizeof...(__InputArgTypes) - sizeof...(__Appends), SimpleContainer<__InputArgTypes...> >::type>::function_type
auto_bind( const __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__InputArgTypes...)> & function, __Appends&&... appends )
{
    typedef typename container::subtype<0, sizeof...(__InputArgTypes) - sizeof...(__Appends), SimpleContainer<__InputArgTypes...> >::type _OutputArgs;
    return details::auto_binder<_OutputArgs::size_>::auto_bind_append( _OutputArgs(), function, std::forward<__Appends>( appends )... );
}

/*
template
<
    class __ReturnType,
    class... __InputArgTypes,
    class... __Prepends,
    typename std::enable_if<(sizeof...(__Prepends) > 0), int>::type = 0
>
typename details::make_function<__ReturnType, typename container::subtype<0, sizeof...(__InputArgTypes) - sizeof...(__Prepends), SimpleContainer<__InputArgTypes...> >::type>::function_type
bind_front( const __AUTO_BIND_FUNCTION_TYPE<__ReturnType(__InputArgTypes...)> & function, __Prepends&&... prepends )
{
    typedef typename container::subtype<sizeof...(__Prepends), sizeof...(__InputArgTypes) - sizeof...(__Prepends), SimpleContainer<__InputArgTypes...> >::type _OutputArgs;
    return details::auto_binder<_OutputArgs::size_>::auto_bind_prepend( _OutputArgs(), function, std::forward<__Prepends>( prepends )... );
}
*/
} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_AUTOBIND_H_

/*###################################################################*/
