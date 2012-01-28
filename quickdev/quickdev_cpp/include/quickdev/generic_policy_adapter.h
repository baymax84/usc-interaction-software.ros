/***************************************************************************
 *  include/quickdev/generic_policy_adapter.h
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

#ifndef QUICKDEVCPP_QUICKDEV_GENERICPOLICYADAPTER_H_
#define QUICKDEVCPP_QUICKDEV_GENERICPOLICYADAPTER_H_

#include <quickdev/macros.h>
#include <quickdev/container.h>
#include <quickdev/policy.h>
#include <utility>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

namespace policy
{
struct AllPolicies{};
typedef AllPolicies ALL_POLICIES;
typedef AllPolicies ALL;
}

template<class __Policy>
struct is_initializeable
{
    static const bool value = false;
};

//! \brief Construct a policy adapter that inherits each of a variable set of policies
/*! \note All policies in the inheritance group take the same set of arguments during construction \see GenericPolicyAdapter(). */
/*! \tparam __Policies the variadic template of policy types to inherit from */
template<class... __Policies>
class GenericPolicyAdapter : public __Policies...
{
public:
    //! \brief Construct a set of policies from a single variable argument
    /*! \note Ideally, we'd be able to pass in a variadic template of types, but gcc can't compile this; "invalid use of pack expansion expression"
     *  \code
     *  template<class... __Args>
     *  GenericPolicyAdapter( __Args... args ) : __Policies( args... )
     *  \endcode
     *  Instead, we pass only one argument.
     *  \see http://stackoverflow.com/questions/7694201/unpacking-parameter-pack-of-args-into-the-constructor-of-each-class-defined-in-a
     *  \tparam __Arg the type of the argument to be passed to all parent policies
     *  \param arg the argument to pass to all parent policies
     *  \note Given the above, arg is almost always a ros::NodeHandle */
    template<class __Arg>
    GenericPolicyAdapter( __Arg & arg )
    :
        __Policies( arg )...
    {

    }

    template<
        bool __ForceInit__,
        class __Policy,
        typename std::enable_if<(!__Policy::IS_INITIALIZEABLE_), int>::type = 0,
        class... __Args
    >
    void tryInit( __Args&&... args ){}

    // for all initable policies
    template<
        bool __ForceInit__,
        class __Policy,
        typename std::enable_if<(__Policy::IS_INITIALIZEABLE_ && __ForceInit__ == true), int>::type = 0,
        class... __Args
    >
    void tryInit( __Args&&... args )
    {
        printPolicyActionStart( "initialize", &__Policy::getInstance() );
        __Policy::init( args... );
        printPolicyActionDone( "initialize", &__Policy::getInstance() );
    }

    // for all initable policies
    template<
        bool __ForceInit__,
        class __Policy,
        typename std::enable_if<(__Policy::IS_INITIALIZEABLE_ && __ForceInit__ == false), int>::type = 0,
        class... __Args
    >
    void tryInit( __Args&&... args )
    {
        if( __Policy::getInitialized() ) return;

        tryInit<true, __Policy>( args... );
    }

    // initialize the first policy in the subset
    // recurse through the remaining policies in the subset
    template<
        bool __ForceInit__,
        class __PoliciesSubset,
        typename std::enable_if<(__PoliciesSubset::size_ > 0), int>::type = 0,
        class... __Args
    >
    void initRec( __Args&&... args )
    {
        tryInit<__ForceInit__, typename container::traits<__PoliciesSubset>::_Front>( args... );
        initRec<__ForceInit__, typename container::traits<__PoliciesSubset>::_Tail>( args... );
    }

    template<
        bool __ForceInit__,
        class __PoliciesSubset,
        typename std::enable_if<(__PoliciesSubset::size_ == 0), int>::type = 0,
        class... __Args
    >
    void initRec( __Args&&... args ) {}

    // Do any post-construction initialization. Note that all policies
    // receive the same set of args.
    template<class... __Args>
    void initAll( __Args&&... args )
    {
        initRec<false, Container<__Policies...> >( args... );
    }

    // Do any post-construction initialization. Note that all policies
    // receive the same set of args. Force re-initialization.
    template<class... __Args>
    void forceInitAll( __Args&&... args )
    {
        initRec<true, Container<__Policies...> >( args... );
    }

    template<
        class... __MPolicies,
        typename std::enable_if<(std::is_same<typename variadic::element<0, __MPolicies...>::type, policy::AllPolicies>::value), int>::type = 0,
        class... __Args
    >
    void initPolicies( __Args&&... args )
    {
        initRec<false, Container<__Policies...> >( args... );
    }

    template<
        class... __MPolicies,
        typename std::enable_if<(!std::is_same<typename variadic::element<0, __MPolicies...>::type, policy::AllPolicies>::value), int>::type = 0,
        class... __Args
    >
    void initPolicies( __Args&&... args )
    {
        initRec<false, Container<__MPolicies...> >( args... );
    }

    template<
        class... __MPolicies,
        typename std::enable_if<(std::is_same<typename variadic::element<0, __MPolicies...>::type, policy::AllPolicies>::value), int>::type = 0,
        class... __Args
    >
    void forceInitPolicies( __Args&&... args )
    {
        initRec<true, Container<__Policies...> >( args... );
    }

    template<
        class... __MPolicies,
        typename std::enable_if<(!std::is_same<typename variadic::element<0, __MPolicies...>::type, policy::AllPolicies>::value), int>::type = 0,
        class... __Args
    >
    void forceInitPolicies( __Args&&... args )
    {
        initRec<true, Container<__MPolicies...> >( args... );
    }
};

}

#endif // QUICKDEVCPP_QUICKDEV_GENERICPOLICYADAPTER_H_
