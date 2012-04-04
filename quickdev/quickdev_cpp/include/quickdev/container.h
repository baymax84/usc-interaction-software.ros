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

namespace variadic
{

//! Access the type of a variadic element
/*! Specialization for non-empty containers
 *  \tparam I the index at which the desired type is defined
 *  \tparam T the type of the variadic to use */
template<unsigned int I, class... T>
struct element;

//! Access the type of a variadic element
/*! Specialization for empty containers
 *  \tparam I the index at which the desired type is defined */
template<unsigned int I>
struct element<I> {};

//! Access the type of a variadic element
/*! Recursive implementation; general case
 *  \tparam I the index at which the desired type is defined
 *  \tparam __Head the type of the element at the front of the subset of types
 *  \tparam __Tail the types of remaining elements in the subset of types */
template<unsigned int I, class __Head, class... __Tail>
struct element<I, __Head, __Tail...> : variadic::element<I-1, __Tail...>
{
    //
};

//! Access the type of a variadic element
/*! Recursive implementation; base case
 *  \tparam __Head the type of the element at the front of the subset of types
 *  \tparam __Tail the types of remaining elements in the subset of types */
template<class __Head, class... __Tail>
struct element<0, __Head, __Tail...>
{
    typedef __Head type;
};

// #############################################################################################################################################

template<unsigned int __Index__, class __Type, class... __Types>
static typename std::enable_if<(__Index__ == 0), typename variadic::element<__Index__, __Type, __Types...>::type>::type &
at_rec( __Type & type, __Types&&... types )
{
    return type;
}

template<unsigned int __Index__, class __Type, class... __Types>
static typename std::enable_if<(__Index__ > 0), typename variadic::element<__Index__, __Type, __Types...>::type>::type &
at_rec( __Type & type, __Types&&... types )
{
    return variadic::at_rec<__Index__ - 1>( std::forward<__Types>( types )... );
}

template<unsigned int __Index__, class... __Types>
static typename variadic::element<__Index__, __Types...>::type &
at( __Types&&... types )
{
    return variadic::at_rec<__Index__>( std::forward<__Types>( types )... );
}

} // variadic

// #############################################################################################################################################

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

// #############################################################################################################################################
//! A variadic Container with no storage
/*! Specialization for a non-empty container */
template<class... __Types>
struct SimpleContainer
{
    //! number of elements
    static unsigned int const size_ = sizeof...( __Types );
    //! front index
    static unsigned int const front_ = 0;
    //! back index
    static unsigned int const back_ = size_ - 1;
};

//! A variadic Container with no storage
/*! Specialization for an empty container */
template<>
struct SimpleContainer<>
{
    //! number of elements
    static unsigned int const size_ = 0;
    //! front index
    static unsigned int const front_ = 0;
    //! back index
    static unsigned int const back_ = 0;
};

//! A variadic Container with std::tuple<> storage
/*! Specialization for a non-empty container
 *  \tparam __Types the types the container will store */
template<class... __Types>
struct Container : SimpleContainer<__Types...>
{
    //! internal storage type
    typedef std::tuple<__Types...> _Storage;
    //! internal storage
    _Storage values_;

    //! Constructor for one or more values
    /*! enabled only if __Types consists of at least one type */
    template<class... __MTypes, typename std::enable_if<(sizeof...(__MTypes) > 0 && std::is_same<SimpleContainer<__Types...>, SimpleContainer<__MTypes...> >::value), int>::type = 0>
    Container( __MTypes&&... values )
    :
        values_( std::make_tuple( std::forward<__MTypes>( values )... ) )
    {
        //
    }

    //! Empty constructor
    Container(){}
};

// #############################################################################################################################################
//! Create a Container from a variadic list
/*! \tparam __Types the types of the values to store
 *  \param types the values to store
 *  \return a container wrapping the given values */
template<class... __Types>
static Container<__Types...> make_container( __Types&&... types )
{
    return quickdev::Container<__Types...>( std::forward<__Types>( types )... );
}

//! Create a SimpleContainer from a variadic list
/*! \tparam __Types the types of the values to store
 *  \param types the values to store
 *  \return a SimpleContainer wrapping the given values */
template<class... __Types>
static Container<__Types...> make_simple_container( __Types&&... types )
{
    return quickdev::SimpleContainer<__Types...>();
}

} // quickdev

namespace container
{

// #############################################################################################################################################
//! Access the type of a Container element
/*! Specialization for non-empty containers
 *  \tparam I the index at which the desired type is defined
 *  \tparam T the type of the container to use */
template<unsigned int I, class T>
struct element;

//! Access the type of a Container element
/*! Specialization for empty containers
 *  \tparam I the index at which the desired type is defined */
template<unsigned int I>
struct element<I, quickdev::SimpleContainer<> > {};

//! Access the type of a Container element
/*! Recursive implementation; general case
 *  \tparam I the index at which the desired type is defined
 *  \tparam __Head the type of the element at the front of the subset of types
 *  \tparam __Tail the types of remaining elements in the subset of types */
template<unsigned int I, class __Head, class... __Tail>
struct element<I, quickdev::SimpleContainer<__Head, __Tail...> > : container::element<I-1, quickdev::SimpleContainer<__Tail...> >
{
    //
};

//! Access the type of a Container element
/*! Recursive implementation; base case
 *  \tparam __Head the type of the element at the front of the subset of types
 *  \tparam __Tail the types of remaining elements in the subset of types */
template<class __Head, class... __Tail>
struct element<0, quickdev::SimpleContainer<__Head, __Tail...> >
{
    typedef __Head type;
};

// #############################################################################################################################################
//! Store useful traits about Container elements
/*! Specialization for non-empty containers
 *  \tparam __ContainerSize__ the size of the given container (for specialization)
 *  \tparam __Container the container for which types should be defined */
template<unsigned int __ContainerSize__, class __Container>
struct elem_traits_helper
{
    //! The type of the element at the front of the container
    typedef typename container::element<__Container::front_, __Container>::type _Front;
    //! The type of the element at the back of the container
    typedef typename container::element<__Container::back_, __Container>::type _Back;
};

//! Store useful traits about Container elements
/*! Specialization for empty containers
 *  \tparam __Container the container for which types should be defined */
template<class __Container>
struct elem_traits_helper<0, __Container>
{

};

//! Store useful traits about Container elements
/*! \tparam __Container the container for which types should be defined */
template<class __Container>
struct elem_traits : elem_traits_helper<__Container::size_, __Container>
{
    //
};

// #############################################################################################################################################
//! Access Container contents
/*! \tparam __Index__ the index of the element to return
 *  \param container the container to read values from
 *  \return the item at an arbitrary index */
template<unsigned int __Index__, class __Container>
static typename container::element<__Index__, __Container>::type at( __Container const & container )
{
    return std::get<__Index__>( container.values_ );
}

//! Access Container contents
/*! \param container the container to read values from
 *  \return the item at the front index */
template<class __Container>
static typename container::elem_traits<__Container>::_Front front( __Container const & container )
{
    return container::at<__Container::front_>( container );
}

//! Access Container contents
/*! \param container the container to read values from
 *  \return the item at the back index */
template<class __Container>
static typename container::elem_traits<__Container>::_Back back( __Container const & container )
{
    return container::at<__Container::back_>( container );
}

// #############################################################################################################################################
//! Calculate the subset of the types of a Container
/*! Recursive implementation; general case
 *  \tparam __EndIndex__ the last index in the subset
 *  \tparam __StartIndex__ the first index in the subset
 *  \tparam __Container the type of the container to read types from
 *  \tparam __Types the list of types accumulated during recursion */
template<unsigned int __EndIndex__, unsigned int __StartIndex__, class __Container, class... __Types>
struct subtype_rec : container::subtype_rec<__EndIndex__ - 1, __StartIndex__, __Container, typename container::element<__EndIndex__, __Container>::type, __Types...>
{
    //
};

//! Calculate the subset of the types of a Container
/*! Recursive implementation; base case
 *  \tparam __StartIndex__ the first index in the subset
 *  \tparam __Container the type of the container to read types from
 *  \tparam __Types the list of types accumulated during recursion */
template<unsigned int __StartIndex__, class __Container, class... __Types>
struct subtype_rec<__StartIndex__, __StartIndex__, __Container, __Types...>
{
    //! The type of the specified subset of the given container
    typedef quickdev::SimpleContainer<typename container::element<__StartIndex__, __Container>::type, __Types...> type;
};

//! Calculate the subset of the types of a Container
/*! Specialization for non-zero items
 *  \tparam __StartIndex__ the first index in the subset
 *  \tparam __NumItems__ the length of the subset
 *  \tparam __Container the type of the container to read types from */
template<unsigned int __StartIndex__, unsigned int __NumItems__, class __Container>
struct subtype : container::subtype_rec<__StartIndex__ + __NumItems__ - 1, __StartIndex__, __Container>
{
    //
};

//! Calculate the subset of the types of a Container
/*! Specialization zero items
 *  \tparam __StartIndex__ the first index in the subset
 *  \tparam __Container the type of the container to read types from */
template<unsigned int __StartIndex__, class __Container>
struct subtype<__StartIndex__, 0, __Container>
{
    typedef quickdev::SimpleContainer<> type;
};

// #############################################################################################################################################
//! Store useful traits about a Container
/*! Specialization for non-empty containers
 *  \tparam __ContainerSize__ the size of the given container (for specialization)
 *  \tparam __Container the container for which types should be defined */
template<unsigned int __ContainerSize__, class __Container>
struct traits_helper
{
    //! the type of the container wrapping the front value of the given container
    typedef quickdev::SimpleContainer<typename container::elem_traits<__Container>::_Front> _Head;
};

//! Store useful traits about a Container
/*! Specialization for empty containers
 *  \tparam __Container the container for which types should be defined */
template<class __Container>
struct traits_helper<0, __Container>
{
    //! the type of the container wrapping the front value of the given container
    typedef quickdev::SimpleContainer<> _Head;
};

//! Store useful traits about a Container
/*! \tparam __Container the container for which types should be defined */
template<class __Container>
struct traits : container::traits_helper<__Container::size_, __Container>, container::elem_traits<__Container>
{
    //! the type of the container wrapping the remaining values of the given container
    typedef typename container::subtype<1, __Container::back_, __Container>::type _Tail;
};

// #############################################################################################################################################
//! Print the contents of a Container
/*! Recursive implementation; general case
 *  \param container the container to print */
template<unsigned int __CurrentIndex__, class... __Types>
static typename std::enable_if<(__CurrentIndex__ == sizeof...( __Types ) - 1 ), void>::type
print_rec( const quickdev::Container<__Types...> & container )
{
    std::cout << container::at<__CurrentIndex__>( container ) << std::endl;
}

//! Print the contents of a Container
/*! Recursive implementation; base case
 *  \param container the container to print */
template<unsigned int __CurrentIndex__, class... __Types>
static typename std::enable_if<(__CurrentIndex__ < sizeof...( __Types ) - 1 ), void>::type
print_rec( const quickdev::Container<__Types...> & container )
{
    std::cout << container::at<__CurrentIndex__>( container ) << std::endl;
    container::print_rec<__CurrentIndex__ + 1>( container );
}

//! Print the contents of a Container
/*! General case
 *  \param container the container to print */
template<class... __Types>
static typename std::enable_if<(sizeof...(__Types) > 0), void>::type
print( const quickdev::Container<__Types...> & container )
{
    container::print_rec<0>( container );
}

//! Print the contents of a Container
/*! Specialization for empty container
 *  \param container the container to print */
template<class... __Types>
static typename std::enable_if<(sizeof...(__Types) == 0), void>::type
print( const quickdev::Container<__Types...> & container )
{
    std::cout << "<empty container>" << std::endl;
}

// #############################################################################################################################################
//! Get a subset of the contents of a Container
/*! Recursive implementation; general case
 *  \tparam __EndIndex__ the last index in the subset
 *  \tparam __StartIndex__ the first index in the subset */
template<unsigned int __EndIndex__, unsigned int __StartIndex__>
struct subset_rec
{
    /* this kills the compiler...
    template<class __AllTypesContainer, class... __Types>
    static auto
    exec( __AllTypesContainer const & container, __Types&&... types )
    -> decltype( container_copy_rec<I - 1, __StartIndex__>::exec( container, std::get<I>( container.values_ ), types... ) )*/

    //! Get a subset of the contents of a Container
    /*! \param container the container to read values from
     *  \param types the values accumulated during recursion
     *  \return the specified subset of the given container */
    template<class __Container, class... __Types>
    static typename container::subtype_rec<__EndIndex__, __StartIndex__, __Container, __Types...>::type
    exec( __Container const & container, __Types&&... types )
    {
        return container::subset_rec<__EndIndex__ - 1, __StartIndex__>::exec( container, container::at<__EndIndex__>( container ), types... );
    }
};

//! Get a subset of the contents of a Container
/*! Recursive implementation; base case
 *  \tparam __StartIndex__ the first index in the subset */
template<unsigned int __StartIndex__>
struct subset_rec<__StartIndex__, __StartIndex__>
{
    //! Get a subset of the contents of a Container
    /*! \param container the container to read values from
     *  \param types the values accumulated during recursion
     *  \return the specified subset of the given container */
    template<class __Container, class... __Types>
    static quickdev::Container<typename container::element<__StartIndex__, __Container>::type, __Types...>
    exec( __Container const & container, __Types&&... types )
    {
        return quickdev::make_container( container::at<__StartIndex__>( container ), types... );
    }
};


//! Get a subset of the contents of a Container
/*! \tparam __StartIndex__ the first index in the subset
 *  \tparam __NumItems__ the length of the subset
 *  \param container the container to read values from
 *  \return the specified subset of the given container */
template<unsigned int __StartIndex__, unsigned int __NumItems__, class __Container>
static typename container::subtype<__StartIndex__, __NumItems__, __Container>::type subset( __Container const & container )
{
    return container::subset_rec<__StartIndex__ + __NumItems__ - 1, __StartIndex__>::exec( container );
}

// #############################################################################################################################################
//! Get a container with the first type popped off the front
/*! \param container the container to read values from
 *  \return a copy of the container with the first type popped off */
template<class __Container>
static typename container::subtype<1, __Container::back_, __Container>::type pop_front( __Container const & container )
{
    return container::subset<1, __Container::back_>( container );
}

// #############################################################################################################################################
//! Get a container with the last type popped off the back
/*! \param container the container to read values from
 *  \return a copy of the container with the last type popped off */
template<class __Container>
static typename container::subtype<0, __Container::back_, __Container>::type pop_back( __Container const & container )
{
    return container::subset<0, __Container::back_>( container );
}

// #############################################################################################################################################
//! Return the head of the container
/*! \param container the container to read values from
 *  \return the head of the container */
template<class __Container>
static typename container::traits<__Container>::_Head head( __Container const & container )
{
    return container::subset<__Container::front_, 1>( container );
}

// #############################################################################################################################################
//! Return the tail of the container
/*! \param container the container to read values from
 *  \return the tail of the container */
template<class __Container>
static typename container::traits<__Container>::_Tail tail( __Container const & container )
{
    return container::subset<1, __Container::back_>( container );
}

// #############################################################################################################################################
//! Access the Container type that results from pushing the given type onto the front of the given Container
template<class __Container, class __NewType>
struct push_front_type;

//! Access the Container type that results from pushing the given type onto the front of the given Container
template<class __NewType, class... __Types>
struct push_front_type<quickdev::SimpleContainer<__Types...>, __NewType>
{
    typedef quickdev::SimpleContainer<__NewType, __Types...> type;
};

// #############################################################################################################################################
//! Access the Container type that results from pushing the given type onto the back of the given Container
template<class __Container, class __NewType>
struct push_back_type;

//! Access the Container type that results from pushing the given type onto the back of the given Container
template<class __NewType, class... __Types>
struct push_back_type<quickdev::SimpleContainer<__Types...>, __NewType>
{
    typedef quickdev::SimpleContainer<__Types..., __NewType> type;
};

// #############################################################################################################################################
template<unsigned int __EndSize__>
struct push_back_rec
{
    template<class __Container, class... __Types>
    static typename container::push_back_type<__Container, typename container::elem_traits<quickdev::Container<__Types...> >::_Back>::type
    exec( __Container const & container, __Types&&... types )
    {
        // push the items of container in order onto types...
        return container::push_back_rec<__EndSize__ - 1>::exec( container, container::at<__EndSize__ - 2>( container ), std::forward<__Types>( types )... );
    }
};

template<>
struct push_back_rec<1>
{
    template<class __Container, class... __Types>
    static quickdev::Container<__Types...>
    exec( __Container const & container, __Types&&... types )
    {
        return quickdev::make_container( std::forward<__Types>( types )... );
    }
};

//! Return the result of pushing the given value onto the back of the given Container
/*! \param container the container to use for the operation
 *  \param type the value to push onto the back of the container
 *  \return the result of pushing the given value onto the back of the given Container */
template<class __Container, class __Type>
static typename container::push_back_type<__Container, __Type>::type
push_back( __Container const & container, __Type const & type )
{
    return container::push_back_rec<__Container::size_ + 1>::exec( container, type );
}

// #############################################################################################################################################
template<unsigned int __EndSize__>
struct push_front_rec
{
    template<class __Container, class __Type, class... __Types>
    static typename container::push_front_type<__Container, __Type>::type
    exec( __Container const & container, __Type const & type, __Types&&... types )
    {
        // push the items of container in order onto types ending with type
        return container::push_front_rec<__EndSize__ - 1>::exec( container, type, container::at<__EndSize__ - 2>( container ), std::forward<__Types>( types )... );
    }
};

template<>
struct push_front_rec<1>
{
    template<class __Container, class __Type, class... __Types>
    static quickdev::Container<__Type, __Types...>
    exec( __Container const & container, __Type const & type, __Types&&... types )
    {
        return quickdev::make_container( type, std::forward<__Types>( types )... );
    }
};

//! Return the result of pushing the given value onto the front of the given Container
/*! \param container the container to use for the operation
 *  \param type the value to push onto the front of the container
 *  \return the result of pushing the given value onto the front of the given Container */
template<class __Container, class __Type>
static typename container::push_front_type<__Container, __Type>::type
push_front( __Container const & container, __Type const & type )
{
    return container::push_front_rec<__Container::size_ + 1>::exec( container, type );
}

// #############################################################################################################################################
//! Return the type of the union of the given containers
template<class __Container1, class __Container2>
struct combine_type;

//! Return the type of the union of the given containers
template<class... __Types1, class... __Types2>
struct combine_type<quickdev::SimpleContainer<__Types1...>, quickdev::SimpleContainer<__Types2...> >
{
    typedef quickdev::SimpleContainer<__Types1..., __Types2...> type;
};

// #############################################################################################################################################
template<unsigned int __Index__>
struct combine_rec
{
    template<class __Container1, class __Container2, class... __Types>
    static typename std::enable_if<(__Index__ > __Container1::size_), typename container::combine_type<__Container1, __Container2>::type>::type
    exec( __Container1 const & container1, __Container2 const & container2, __Types&&... types )
    {
        return container::combine_rec<__Index__ - 1>::exec( container1, container2, container::at<__Index__ - __Container1::size_ - 1>( container2 ), std::forward<__Types>( types )... );
    }

    template<class __Container1, class __Container2, class... __Types>
    static typename std::enable_if<(__Index__ <= __Container1::size_), typename container::combine_type<__Container1, __Container2>::type>::type
    exec( __Container1 const & container1, __Container2 const & container2, __Types&&... types )
    {
        return container::combine_rec<__Index__ - 1>::exec( container1, container2, container::at<__Index__ - 1>( container1 ), std::forward<__Types>( types )... );
    }
};

template<>
struct combine_rec<0>
{
    template<class __Container1, class __Container2, class... __Types>
    static typename container::combine_type<__Container1, __Container2>::type exec( __Container1 const & container1, __Container2 const & container2, __Types&&... types )
    {
        return quickdev::make_container( std::forward<__Types>( types )... );
    }
};

//! Return the union of the given containers
/*! \param container1 the first container
 *  \param container2 the second container
 *  \return a new container composed of { <container1 values...>, <container2 values...> } */
template<class __Container1, class __Container2>
static typename container::combine_type<__Container1, __Container2>::type combine( __Container1 const & container1, __Container2 const & container2 )
{
    return container::combine_rec<__Container1::size_+ __Container2::size_>::exec( container1, container2 );
};

} // container

#endif // QUICKDEVCPP_QUICKDEV_CONTAINER_H_
