/***************************************************************************
 *  include/quickdev/policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_POLICY_H_
#define QUICKDEVCPP_QUICKDEV_POLICY_H_

#include <quickdev/console.h>
#include <quickdev/generic_policy_adapter.h>
#include <quickdev/type_utils.h>
#include <quickdev/param_reader.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

template<class __Policy>
static void printPolicyAction( std::string const & action, __Policy const * const policy )
{
    PRINT_INFO( "[ %s ] on policy: [ %s ]", action.c_str(), __Policy::name().c_str() );
}

template<class __Policy>
static void printPolicyActionStart( std::string const & action, __Policy const * const policy )
{
    printPolicyAction( ">>>>> Start: " + action, policy );
}

template<class __Policy>
static void printPolicyActionDone( std::string const & action, __Policy const * const policy )
{
    printPolicyAction( "<<<<< Done: " + action, policy );
}

namespace policy
{
template<class __ParamType, class... __Args>
static __ParamType readPolicyParam( ros::NodeHandle & nh, std::string const & meta_param_key, std::string const & ros_param_name, __ParamType const & default_value, __Args&&... args )
{
    __ParamType param_value;
    if( tryGetMetaParam<__ParamType>( meta_param_key, param_value, std::forward<__Args>( args )... ) ) return param_value;
    return ros::ParamReader<__ParamType, 1>::readParam( nh, ros_param_name, default_value );
}

template<class __ParamType, class... __Args>
static __ParamType readPolicyParamWithId( std::string const & meta_param_key, unsigned int const & id, __ParamType const & default_value, __Args&&... args )
{
    return getMetaParamDef<__ParamType>( make_key( meta_param_key, id ), default_value, std::forward<__Args>( args )... );
}

template<class __ParamType, class... __Args>
static __ParamType readPolicyParamAuto( ros::NodeHandle & nh, bool const & enable_key_ids, std::string const & meta_param_key, unsigned int const & id, std::string const & ros_param_name, __ParamType const & default_value, __Args&&... args )
{
    if( enable_key_ids )
    {
        return readPolicyParamWithId( meta_param_key, id, default_value, std::forward<__Args>( args )... );
    }
    return readPolicyParam( nh, meta_param_key, ros_param_name, default_value, std::forward<__Args>( args )... );
}

} // policy

class Policy
{
public:
    static bool const IS_INITIALIZEABLE_ = false;
    static inline std::string name() { return "Base"; }

    inline Policy & getInstance() { return *this; }

    template<class... __Args>
    Policy( __Args&&... args )
    {
        // discard args... but don't fail to compile if anything is passed in
        PRINT_INFO( "########## Policy Group ##########" );
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }
};

}

#endif // QUICKDEVCPP_QUICKDEV_POLICY_H_
