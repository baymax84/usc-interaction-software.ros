/***************************************************************************
 *  include/quickdev/convolved_struct.h
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

#ifndef QUICKDEVCPP_QUICKDEV_CONVOLVEDSTRUCT_H_
#define QUICKDEVCPP_QUICKDEV_CONVOLVEDSTRUCT_H_

#include <quickdev/macros.h>
#include <quickdev/container.h>

#define DECLARE_CONVOLVED_STRUCT( Name, __Parent ) \
typedef __Parent _##Name##Parent; \
struct Name : __Parent

#define DECLARE_CONVOLVED_STRUCT_BASE( Name ) \
DECLARE_CONVOLVED_STRUCT( Name, quickdev::ConvolvedBase )

#define DECLARE_CONVOLVED_STRUCT_TYPES( Name, types... ) \
typedef _##Name##Parent _Parent; \
typedef quickdev::Container<types> _TypesContainer

// we have four constructors here
// - the first constructor
//   - empty constructor
// - the second constructor
//   - enabled if
//     - the number of args passed matches the number of args required by the given struct and its parent and
//     - the first arg is not a parent reference and
//     - the appropriate subset of args passed matches the types of args required by the given struct
//   - calls the top-level constructor and passes an appropriate parent instance, a container specifying the arg offset to use (the number of parent args), and the given args
// - the third constructor
//   - enabled if
//     - the number of args passed exactly matches the number required by the given struct
//     - the types of the args passed exactly matches the types required by the given struct
//   - calls the top-level constructor and passes a the given parent instance, a container specifying the arg offset to use (zero), and the given args
// - the fourth constructor
//   - copies the given parent and reads the arg offset to use; allows the user to set all member variables with INST_CONVOLVED_STRUCT_VAR( name, offset )
//
#define INST_CONVOLVED_STRUCT( Name ) \
Name(){} \
template< \
    class... __Args, \
    typename std::enable_if<(sizeof...(__Args) > 0 && sizeof...(__Args) >= _TypesContainer::size_ + _Parent::_TypesContainer::size_), int>::type = 0, \
    typename std::enable_if<(!std::is_same<typename variadic::element<0, __Args...>::type, _Parent>::value), int>::type = 0, \
    typename std::enable_if<(std::is_same<typename container::subtype<_Parent::_TypesContainer::size_, _TypesContainer::size_, quickdev::Container<__Args...> >::type, _TypesContainer>::value), int>::type = 0> \
Name( __Args&&... args ){ *this = Name( _Parent( std::forward<__Args>( args )... ), _Parent::_TypesContainer(), std::forward<__Args>( args )... ); } \
 \
template< \
    class... __Args, \
    typename std::enable_if<(sizeof...(__Args) > 0 && sizeof...(__Args) == _TypesContainer::size_), int>::type = 0, \
    typename std::enable_if<(std::is_same<quickdev::Container<__Args...>, _TypesContainer>::value), int>::type = 0> \
Name( _Parent const & parent, __Args&&... args ){ *this = Name( parent, quickdev::Container<>(), std::forward<__Args>( args )... ); } \
 \
Name( _Parent const & parent ) : _Parent( parent ){} \
 \
template<class... __OffsetTypes, class... __Args> \
Name( _Parent const & parent, const quickdev::Container<__OffsetTypes...> & offset_container, __Args&&... args ) : _Parent( parent )

// allows the user to set the value of member vars by name and offset during construction
// ie: INST_CONVOLVED_STRUCT( SomeStruct ),
//       INST_CONVOLVED_STRUCT_VAR( a, 0 ),
//       INST_CONVOLVED_STRUCT_VAR( b, 1 ),
//       INST_CONVOLVED_STRUCT_VAR( c, 2 )
//     {}
//
#define INST_CONVOLVED_STRUCT_VAR( index, name ) \
name( variadic::at<quickdev::Container<__OffsetTypes...>::size_ + index>( std::forward<__Args>( args )... ) )

// Using the above, the user can do:
//
// struct A{};
// struct B{};
//
// DECLARE_CONVOLVED_STRUCT_BASE( SomeBase )
// {
//    A a;
//    B b;
//
//    DECLARE_CONVOLVED_STRUCT_TYPES( SomeBase, A, B );
//
//    INST_CONVOLVED_STRUCT( SomeBase ),
//      INST_CONVOLVED_STRUCT_VAR( a, 0 ),
//      INST_CONVOLVED_STRUCT_VAR( b, 1 )
//    {}
// };
//
// --------------------
//
// struct C{};
// struct D{};
//
// DECLARE_CONVOLVED_STRUCT( SomeDerived, SomeBase )
// {
//    C c;
//    D d;
//
//    DECLARE_CONVOLVED_STRUCT_TYPES( SomeDerived, C, D );
//
//    INST_CONVOLVED_STRUCT( SomeDerived ),
//      INST_CONVOLVED_STRUCT_VAR( c, 0 ),
//      INST_CONVOLVED_STRUCT_VAR( d, 1 )
//    {}
// };
//
// --------------------
//
// A a;
// B b;
// C c;
// D d;
//
// SomeBase base1( a, b );
// SomeBase base2( a, B() );
// SomeDerived derived1( a, b, c, d );
// SomeDerived derived2( a, B(), c, D() );
// SomeDerived derived3( SomeBase( a, b ), c, d )
// SomeDerived derived4( SomeBase( a, B() ), c, D() )
// SomeDerived derived5( base1, c, d )
// SomeDerived derived6( base2, c, D() )
//
// And so on.
//
// #############################################################################################################################################

#define INST_CONVOLVED_STRUCT2( Name ) \
template< \
    class... __Args, \
    typename std::enable_if<(sizeof...(__Args) >= _TypesContainer::size_), int>::type = 0, \
    typename std::enable_if<(!std::is_same<typename variadic::element<0, __Args...>::type, _Parent>::value), int>::type = 0, \
    typename std::enable_if<(std::is_same<typename container::subtype<_Parent::_TypesContainer::size_, _TypesContainer::size_, quickdev::Container<__Args...> >::type, _TypesContainer>::value), int>::type = 0> \
Name( __Args&&... args ) : _Parent( std::forward<__Args>( args )... )

#define INST_CONVOLVED_STRUCT_LVAL( Name ) \
template<class... __Args, typename std::enable_if<(sizeof...(__Args) >= _TypesContainer::size_), int>::type = 0, typename std::enable_if<(std::is_same<typename container::subtype<_Parent::_TypesContainer::size_, _TypesContainer::size_, quickdev::Container<__Args&...> >::type, _TypesContainer>::value), int>::type = 0> \
Name( __Args&... args ) : _Parent( std::forward<__Args>( args )... )

#define INST_CONVOLVED_STRUCT_RVAL( Name ) \
template<class... __Args, typename std::enable_if<(sizeof...(__Args) >= _TypesContainer::size_), int>::type = 0, typename std::enable_if<(std::is_same<typename container::subtype<_Parent::_TypesContainer::size_, _TypesContainer::size_, quickdev::Container<__Args&&...> >::type, _TypesContainer>::value), int>::type = 0> \
Name( __Args&&... args ) : _Parent( std::forward<__Args>( args )... )

#define INST_CONVOLVED_STRUCT_VAR2( name, index ) \
name( variadic::at<_Parent::_TypesContainer::size_ + index>( std::forward<__Args>( args )... ) )

// #############################################################################################################################################

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

struct ConvolvedBase
{
    typedef quickdev::Container<> _TypesContainer;

    template<class... __Args>
    ConvolvedBase( __Args... ){}
};

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_CONVOLVEDSTRUCT_H_
