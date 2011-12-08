/***************************************************************************
 *  include/quickdev/container.h
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

#ifndef QUICKDEVCPP_QUICKDEV_CONTAINER_H_
#define QUICKDEVCPP_QUICKDEV_CONTAINER_H_

#include <quickdev/macros.h>

#include <type_traits>
#include <tuple>
#include <ostream>
#include <iostream>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

// #############################################################################################################################################
// Container
//
template<class... __Types>
struct Container
{
    typedef std::tuple<__Types...> _Storage;
    _Storage values_;
    const static unsigned int size_ = sizeof...( __Types );
    const static unsigned int front_ = 0;
    const static unsigned int back_ = size_ - 1;

    Container( __Types... values )
    :
        values_( std::make_tuple( values... ) )
    {
        //
    }
    Container(){}
};

template<>
struct Container<>
{
    const static unsigned int size_ = 0;
    const static unsigned int front_ = 0;
    const static unsigned int back_ = 0;
};

// #############################################################################################################################################
// Create a Container from a variadic list
//
template<class... __Types>
static Container<__Types...> make_container( __Types... types )
{
    return Container<__Types...>( types... );
}

} // quickdev

namespace container
{

// #############################################################################################################################################
// Container element type access
//
template<unsigned int I, class T>
struct element;

template<unsigned int I, class __Head, class... __Tail>
struct element<I, quickdev::Container<__Head, __Tail...> > : container::element<I-1, quickdev::Container<__Tail...> >
{
    //
};

template<class __Head, class... __Tail>
struct element<0, quickdev::Container<__Head, __Tail...> >
{
    typedef __Head type;
};

template<unsigned int I>
struct element<I, quickdev::Container<> > {};

// #############################################################################################################################################
// container_elem traits
//

template<unsigned int __ContainerSize__, class __Container>
struct elem_traits_helper
{
    typedef typename container::element<__Container::front_, __Container>::type _Front;
    typedef typename container::element<__Container::back_, __Container>::type _Back;
};

template<class __Container>
struct elem_traits_helper<0, __Container>
{

};

template<class __Container>
struct elem_traits : elem_traits_helper<__Container::size_, __Container>
{
    //
};

// #############################################################################################################################################
// Common element access
//
template<unsigned int __Index__, class __Container>
static typename container::element<__Index__, __Container>::type at( const __Container & container )
{
    return std::get<__Index__>( container.values_ );
}

template<class __Container>
static typename container::elem_traits<__Container>::_Front front( const __Container & container )
{
    return container::at<__Container::front_>( container );
}

template<class __Container>
static typename container::elem_traits<__Container>::_Back back( const __Container & container )
{
    return container::at<__Container::back_>( container );
}

// #############################################################################################################################################
// Calculate a subset of the types of a Container
//
template<unsigned int __EndIndex__, unsigned int __StartIndex__, class __Container, class... __Types>
struct subtype_rec : container::subtype_rec<__EndIndex__ - 1, __StartIndex__, __Container, typename container::element<__EndIndex__, __Container>::type, __Types...>
{
    //
};

template<unsigned int __StartIndex__, class __Container, class... __Types>
struct subtype_rec<__StartIndex__, __StartIndex__, __Container, __Types...>
{
    typedef quickdev::Container<typename container::element<__StartIndex__, __Container>::type, __Types...> type;
};

template<unsigned int __StartIndex__, unsigned int __NumItems__, class __Container>
struct subtype : container::subtype_rec<__StartIndex__ + __NumItems__ - 1, __StartIndex__, __Container>
{
    //
};

template<unsigned int __StartIndex__, class __Container>
struct subtype<__StartIndex__, 0, __Container>
{
    typedef quickdev::Container<> type;
};

// #############################################################################################################################################
// container traits
//

template<unsigned int __ContainerSize__, class __Container>
struct traits_helper
{
    typedef quickdev::Container<typename container::elem_traits<__Container>::_Front> _Head;
};

template<class __Container>
struct traits_helper<0, __Container>
{
    typedef quickdev::Container<> _Head;
};

template<class __Container>
struct traits : container::traits_helper<__Container::size_, __Container>, container::elem_traits<__Container>
{
    typedef typename container::subtype<1, __Container::back_, __Container>::type _Tail;
};

// #############################################################################################################################################
// Print Container contents
//
template<unsigned int __CurrentIndex__, class... __Types>
static typename std::enable_if<(__CurrentIndex__ == sizeof...( __Types ) - 1 ), void>::type
print_rec( const quickdev::Container<__Types...> & container )
{
    std::cout << container::at<__CurrentIndex__>( container ) << std::endl;
}

template<unsigned int __CurrentIndex__, class... __Types>
static typename std::enable_if<(__CurrentIndex__ < sizeof...( __Types ) - 1 ), void>::type
print_rec( const quickdev::Container<__Types...> & container )
{
    std::cout << container::at<__CurrentIndex__>( container ) << std::endl;
    container::print_rec<__CurrentIndex__ + 1>( container );
}

template<class... __Types>
static typename std::enable_if<(sizeof...(__Types) > 0), void>::type
print( const quickdev::Container<__Types...> & container )
{
    container::print_rec<0>( container );
}

template<class... __Types>
static typename std::enable_if<(sizeof...(__Types) == 0), void>::type
print( const quickdev::Container<__Types...> & container )
{
    std::cout << "<empty container>" << std::endl;
}

// #############################################################################################################################################
// Get a subset of the contents of a Container
//
template<unsigned int __EndIndex__, unsigned int __StartIndex__>
struct subset_rec
{
    /* this kills the compiler...
    template<class __AllTypesContainer, class... __Types>
    static auto
    exec( const __AllTypesContainer & container, __Types... types )
    -> decltype( container_copy_rec<I - 1, __StartIndex__>::exec( container, std::get<I>( container.values_ ), types... ) )*/

    template<class __Container, class... __Types>
    static typename container::subtype_rec<__EndIndex__, __StartIndex__, __Container, __Types...>::type
    exec( const __Container & container, __Types... types )
    {
        return container::subset_rec<__EndIndex__ - 1, __StartIndex__>::exec( container, container::at<__EndIndex__>( container ), types... );
    }
};

template<unsigned int __StartIndex__>
struct subset_rec<__StartIndex__, __StartIndex__>
{
    template<class __Container, class... __Types>
    static quickdev::Container<typename container::element<__StartIndex__, __Container>::type, __Types...>
    exec( const __Container & container, __Types... types )
    {
        return quickdev::make_container( container::at<__StartIndex__>( container ), types... );
    }
};

template<unsigned int __StartIndex__, unsigned int __NumItems__, class __Container>
static typename container::subtype<__StartIndex__, __NumItems__, __Container>::type subset( const __Container & container )
{
    return container::subset_rec<__StartIndex__ + __NumItems__ - 1, __StartIndex__>::exec( container );
}

// #############################################################################################################################################
// Get a container with the first type popped off the front
//
template<class __Container>
static typename container::subtype<1, __Container::back_, __Container>::type pop_front( const __Container & container )
{
    return container::subset<1, __Container::back_>( container );
}

// #############################################################################################################################################
// Get a container with the last type popped off the back
//
template<class __Container>
static typename container::subtype<0, __Container::back_, __Container>::type pop_back( const __Container & container )
{
    return container::subset<0, __Container::back_>( container );
}

} // container

#endif // QUICKDEVCPP_QUICKDEV_CONTAINER_H_
