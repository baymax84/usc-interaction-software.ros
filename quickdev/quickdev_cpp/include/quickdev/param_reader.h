/***************************************************************************
 *  include/quickdev/param_reader.h
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

#ifndef QUICKDEVCPP_QUICKDEV_PARAMREADER_H_
#define QUICKDEVCPP_QUICKDEV_PARAMREADER_H_

#include <quickdev/console.h>
#include <ros/param.h>
#include <ros/node_handle.h>
#include <quickdev/macros.h>
#include <type_traits>
#include <vector>
#include <sstream>

// a class that will read up to __Dim__ paramters with the name format "prefix_|n|postfix_"
// a dim of 0 means "load all matching params that are found, without limit"
// so you can do something like:
// - ParamReader param_reader1( nh, ... ).readParams();
// - ParamReader param_reader2( nh ).readParams( ... );
// - ParamReader param_reader3( nh );
//   param_reader1.readParams( ... )
//   param_reader2.readParams( ... )
//   param_reader3.readParams( ... )

namespace ros
{

template<class __Storage, unsigned int __Dim__ = 0>
class ParamReader
{
public:
    typedef __Storage _Storage;
    typedef std::vector<__Storage> _Array;

    ros::NodeHandle nh_;
    std::string prefix_;
    std::string postfix_;
    unsigned int start_index_;
    _Array params_;

    ParamReader(){}

    ParamReader(
        ros::NodeHandle & nh,
        std::string const & prefix,
        std::string const & postfix = "",
        unsigned int start_index = 1 )
    :
        nh_( nh ),
        prefix_( prefix ),
        postfix_( postfix ),
        start_index_( start_index )
    {
        //
    }

public:
    _Array
        readParams()
    {
        params_ = readParams(
            nh_,
            prefix_,
            postfix_,
            start_index_ );

        return params_;
    }

    static _Array
        readParams(
            ros::NodeHandle & nh,
            std::string const & prefix,
            std::string const & postfix = "",
            unsigned int start_index = 1 )
    {
        return readParams( nh, prefix, postfix, start_index, {} );
    }

    static _Array
        readParams(
            ros::NodeHandle & nh,
            std::string const & prefix,
            std::string const & postfix,
            unsigned int start_index,
            std::initializer_list<__Storage> const & defaults )
    {
        _Array params( defaults.size() );
        std::copy( defaults.begin(), defaults.end(), params.begin() );

        bool new_param_found = true;
        unsigned int n = start_index;
        unsigned int i;

        std::stringstream num_params_ss;
        if( __Dim__ > 0 ) num_params_ss << __Dim__;
        else num_params_ss << "any";

        PRINT_INFO( "Attempting to load [ %s ] parameters in the form [ %s#%s ] starting with index [ %u ]", num_params_ss.str().c_str(), prefix.c_str(), postfix.c_str(), start_index );

        do
        {
            i = n - start_index;
            std::stringstream param_name_ss;
            param_name_ss << prefix << n << postfix;

            __Storage param_value;
            std::string const param_name = param_name_ss.str();

            if( nh.getParam(
                param_name.c_str(),
                param_value ) )
            {
                std::stringstream param_value_ss;
                param_value_ss << param_value;
                PRINT_INFO( ">>> Loaded param [ %s ] with value [ %s ]", param_name.c_str(), param_value_ss.str().c_str() );
                if( params.size() > i ) params[i] = param_value;
                else params.push_back( param_value );
                ++n;
            }
            else
            {
                if( __Dim__ > 0 ) PRINT_WARN( "Only found [ %i/%i ] parameters in array", i, __Dim__ );
                else
                {
                    if( i > 0 ) PRINT_INFO( "Found [ %i ] parameters in array", i );
                    else PRINT_WARN( "No parameters found in array" );
                }
                //PRINT_WARN( "%s[%u:%u]%s", prefix.c_str(), start_index, start_index + __Dim__, postfix.c_str() );
                new_param_found = false;
            }
        }
        while( new_param_found && ( __Dim__ == 0 || n < __Dim__ + start_index ) );
        return params;
    }
};

// a ParamReader of length one can be initialized with a single
// nodehandle and readParam() can be called repeatedly to fetch params
// so you can do something like:
// - ParamReader param_reader1( nodehandle, ... ).getParam();
// - ParamReader param_reader2( nodehandle ).getParam( ... );
// - ParamReader param_reader3( nodehandle );
//   param_reader3.getParam( "param1" );
//   param_reader2.getParam( "param2" );
//   param_reader1.getParam( "param3" );

template<class __Storage>
class ParamReader<__Storage, 1>
{
public:
    ros::NodeHandle nh_;

    std::string param_name_;

    __Storage
        default_value_,
        last_value_;

    ParamReader(){}

    ParamReader(
        ros::NodeHandle & nh,
        std::string const & param_name = "",
        __Storage const & default_value = __Storage() )
    :
        nh_( nh ),
        default_value_( default_value ),
        last_value_( default_value_ )
    {
        //
    }

    __Storage
        readParam()
    {
        return readParam(
            nh_,
            param_name_,
            default_value_ );
    }

    template
    <
        class __MStorage,
        typename std::enable_if<!std::is_same<__MStorage, XmlRpc::XmlRpcValue>::value, int>::type = 0
    >
    static __Storage
        readParam(
            ros::NodeHandle & nh,
            std::string const & param_name,
            __Storage const & default_value = __Storage() )
    {
        __Storage param_value( default_value );

        const bool param_found( tryReadParam( nh, param_name, param_value ) );

        std::stringstream param_value_ss;
        param_value_ss << param_value;

        if( param_found )
        {
            PRINT_INFO( ">>> Using value [ %s ]", param_value_ss.str().c_str() );
        }
        else
        {
            PRINT_WARN( ">>> Defaulting to [ %s ]", param_value_ss.str().c_str() );
        }

        return  param_value;
    }

    template
    <
        class __MStorage,
        typename std::enable_if<std::is_same<__MStorage, XmlRpc::XmlRpcValue>::value, int>::type = 0
    >
    static __Storage
        readParam(
            ros::NodeHandle & nh,
            std::string const & param_name,
            __Storage const & default_value = __Storage() )
    {
        __Storage param_value( default_value );

        const bool param_found( tryReadParam( nh, param_name, param_value ) );

        if( param_found )
        {
            PRINT_INFO( ">>> Found paramter" );
        }
        else
        {
            PRINT_WARN( ">>> Using default value" );
        }

        return  param_value;
    }

    template<class... __Args>
    static __Storage readParam( __Args&&... args )
    {
        return readParam<__Storage>( args... );
    }

    static bool tryReadParam(
        ros::NodeHandle & nh,
        std::string const & param_name,
        __Storage & param_value )
    {
        if( nh.getParam(
            param_name.c_str(),
            param_value ) )
        {
            PRINT_INFO( "Found param [ %s/%s ]",
            nh.getNamespace().c_str(),
            param_name.c_str() );

            return true;
        }

        PRINT_WARN(
            "Could not find param [ %s/%s ]",
            nh.getNamespace().c_str(),
            param_name.c_str() );

        return false;
    }

    static __Storage getXmlRpcValue( XmlRpc::XmlRpcValue & xml_rpc_value, std::string const & member_name, __Storage const & default_value = __Storage() )
    {
        if( xml_rpc_value.hasMember( member_name ) ) return __Storage( xml_rpc_value[member_name] );

        PRINT_WARN( "XmlRpcValue does not contain member %s; returning default", member_name.c_str() );
        return default_value;
    }
};

} // ros

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

namespace param_reader_traits
{

typedef XmlRpc::XmlRpcValue _XmlRpcValue;

template<class __Output>
struct XmlRpcStorageAdapter
{
    typedef __Output _Output;

    static __Output convert( _XmlRpcValue && input )
    {
        return __Output( input );
    }
};

template<class __Storage>
struct XmlRpcStorageAdapter<std::vector<__Storage> >
{
    typedef _XmlRpcValue::ValueArray _XmlRpcType;
    typedef std::vector<__Storage> _Output;

    static _Output convert( _XmlRpcValue && input )
    {
        _Output result( input.size() );

        for( int i = 0; i < input.size(); ++i )
        {
            result[i] = __Storage( input[i] );
        }

        return result;
    }
};

template<class __Storage>
struct XmlRpcStorageAdapter<std::map<std::string, __Storage> >
{
    typedef _XmlRpcValue::ValueStruct _XmlRpcType;
    typedef std::map<std::string, __Storage> _Output;

    static _Output convert( _XmlRpcValue && input )
    {
        _Output result;

        for( auto input_it = input.begin(); input_it != input.end(); ++input_it )
        {
            result[input_it->first] = __Storage( input_it->second );
        }

        return result;
    }
};

} // param_reader_traits


class ParamReader
{
public:
    typedef param_reader_traits::_XmlRpcValue _XmlRpcValue;

protected:
    ros::NodeHandle & nh_;

public:
    ParamReader( ros::NodeHandle && nh )
    :
        nh_( nh )
    {
        //
    }

    template<class __Key, class __Data>
    static std::string toString( std::map<__Key, __Data> const & param )
    {
        return std::string( "std::map" );
    }

    template<class __Data>
    static std::string toString( std::vector<__Data> const & param )
    {
        return std::string( "std::vector" );
    }

    template
    <
        class __Param,
        typename std::enable_if<!(std::is_arithmetic<__Param>::value), int>::type = 0
    >
    static std::string toString( __Param const & param )
    {
        return std::string( "non-arithmetic type" );
    }

    static std::string toString( std::string const & param )
    {
        return param;
    }

    static std::string toString( XmlRpc::XmlRpcValue & param )
    {
        return std::string( "XmlRpcValue" );
    }

    template
    <
        class __Param,
        typename std::enable_if<(std::is_arithmetic<__Param>::value), int>::type = 0
    >
    static std::string toString( __Param const & param )
    {
        std::stringstream ss;
        ss << param;
        return ss.str();
    }

    template
    <
        class __Output,
        typename std::enable_if<( std::is_arithmetic<__Output>::value || std::is_same<std::string, __Output>::value || std::is_same<XmlRpc::XmlRpcValue, __Output>::value ), int>::type = 0
    >
    static __Output readParam( ros::NodeHandle & nh, std::string const & name, __Output const & default_value = __Output() )
    {
        __Output param;
        bool const param_found = nh.getParam( name, param );

        if( param_found )
        {
            PRINT_INFO( "Found param [ %s/%s ] with value [ %s ]", nh.getNamespace().c_str(), name.c_str(), toString( param ).c_str() );
            return param;
        }

        PRINT_WARN( "Param [ %s/%s ] not found; using default value [ %s ]", nh.getNamespace().c_str(), name.c_str(), toString( default_value ).c_str() );
        return default_value;
    }

    template
    <
        class __Output,
        typename std::enable_if<!( std::is_arithmetic<__Output>::value || std::is_same<std::string, __Output>::value || std::is_same<XmlRpc::XmlRpcValue, __Output>::value ), int>::type = 0
    >
    static __Output readParam( ros::NodeHandle & nh, std::string const & name, __Output const & default_value = __Output() )
    {
        _XmlRpcValue param;
        bool const param_found = nh.getParam( name, param );

        if( param_found )
        {
            auto result = param_reader_traits::XmlRpcStorageAdapter<__Output>::convert( param );
            PRINT_INFO( "Found param [ %s/%s ] with value [ %s ]", nh.getNamespace().c_str(), name.c_str(), toString( result ).c_str() );
            return result;
        }

        PRINT_WARN( "Param [ %s/%s ] not found; using default value [ %s ]", nh.getNamespace().c_str(), name.c_str(), toString( default_value ).c_str() );
        return default_value;
    }

    template<class __Output>
    static __Output fromXmlRpcValue( _XmlRpcValue & param )
    {
        return param_reader_traits::XmlRpcStorageAdapter<__Output>::convert( param );
    }

    template<class __Output>
    static __Output getXmlRpcValue( XmlRpc::XmlRpcValue & xml_rpc_value, std::string const & member_name, __Output const & default_value = __Output() )
    {
        if( xml_rpc_value.hasMember( member_name ) ) return __Output( xml_rpc_value[member_name] );

        PRINT_WARN( "XmlRpcValue does not contain member %s; returning default", member_name.c_str() );
        return default_value;
    }

    template<class __Output>
    __Output readParam( std::string const & name, __Output const & default_value = __Output() )
    {
        return readParam<__Output>( nh_, name, default_value );
    }
};

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_PARAMREADER_H_
