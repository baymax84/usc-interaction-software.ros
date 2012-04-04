#ifndef QUICKDEVCPP_QUICKDEV_VARIADIC_H_
#define QUICKDEVCPP_QUICKDEV_VARIADIC_H_

// for std::enable_if
#include <type_traits>
// for std::forward
#include <utility>

namespace variadic
{

// =============================================================================================================================================
//! Access the type of a variadic element
/*! Specialization for non-empty containers
 *  \tparam I the index at which the desired type is defined
 *  \tparam T the type of the variadic to use */
template<unsigned int I, class... T>
struct element;

// =============================================================================================================================================
//! Access the type of a variadic element
/*! Specialization for empty containers
 *  \tparam I the index at which the desired type is defined */
template<unsigned int I>
struct element<I> {};

// =============================================================================================================================================
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

// =============================================================================================================================================
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

// =============================================================================================================================================
template<unsigned int __Index__, class __Type, class... __Types>
static typename std::enable_if<(__Index__ == 0), typename variadic::element<__Index__, __Type, __Types...>::type>::type &
at_rec( __Type & type, __Types&&... types );

// =============================================================================================================================================
template<unsigned int __Index__, class __Type, class... __Types>
static typename std::enable_if<(__Index__ > 0), typename variadic::element<__Index__, __Type, __Types...>::type>::type &
at_rec( __Type & type, __Types&&... types );

// =============================================================================================================================================
template<unsigned int __Index__, class... __Types>
static typename variadic::element<__Index__, __Types...>::type &
at( __Types&&... types );

// #############################################################################################################################################

#include <quickdev/details/variadic_impl.h>

} // variadic

#endif // QUICKDEVCPP_QUICKDEV_VARIADIC_H_
