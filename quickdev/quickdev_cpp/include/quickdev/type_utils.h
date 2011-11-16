/***************************************************************************
 *  include/quickdev/type_utils.h
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

#ifndef QUICKDEVCPP_QUICKDEV_TYPEUTILS_H_
#define QUICKDEVCPP_QUICKDEV_TYPEUTILS_H_

#include <quickdev/console.h>
#include <quickdev/macros.h>
#include <type_traits>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <ros/message.h>
//#include <ros/message_traits.h>

namespace quickdev
{
	struct TYPE_NOT_FOUND{};

	template<class __Desired>
	static __Desired getFirstOfType(){ return TYPE_NOT_FOUND(); }

	// matching value found; return it
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<std::is_same<__Desired, __Current>::value, __Desired>::type
	getFirstOfType( __Current & current, __Rest&&... rest )
	{
		return current;
	}

	// iterate through types in __Rest until a matching type is found
	// this will fail at compile time if no matching type exists
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<!std::is_same<__Desired, __Current>::value, __Desired>::type
	getFirstOfType( __Current & current, __Rest&&... rest )
	{
		return getFirstOfType<__Desired>( rest... );
	}

	template<class __Desired>
	__Desired getMetaParamRec( const std::string & name )
	{
		PRINT_ERROR( ">>> Failed to find key [ %s ]", name.c_str() );
		return __Desired();
	}

	template<class __Desired>
	__Desired getMetaParamDefRec( const std::string & name, const __Desired & default_value )
	{
		std::stringstream ss;
		ss << default_value;
		PRINT_WARN( ">>> Failed to find key [ %s ]; defaulting to [ %s ]", name.c_str(), ss.str().c_str() );
		return default_value;
	}

	// ####

	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(!std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamRec( const std::string & name, const std::string & current_name, __Current & current, __Rest&&... rest );

	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamRec( const std::string & name, const std::string & current_name, __Current & current, __Rest&&... rest );

	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(!std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamDefRec( const std::string & name, const __Desired & default_value, const std::string & current_name, __Current & current, __Rest&&... rest );

	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamDefRec( const std::string & name, const __Desired & default_value, const std::string & current_name, __Current & current, __Rest&&... rest );

	// ####

	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(!std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamRec( const std::string & name, const std::string & current_name, __Current & current, __Rest&&... rest )
	{
		return getMetaParamRec<__Desired>( name, rest... );
	}

	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(!std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamDefRec( const std::string & name, const __Desired & default_value, const std::string & current_name, __Current & current, __Rest&&... rest )
	{
		return getMetaParamDefRec<__Desired>( name, default_value, rest... );
	}

	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamRec( const std::string & name, const std::string & current_name, __Current & current, __Rest&&... rest )
	{
		if( name == current_name )
		{
			std::stringstream ss;
			ss << current;
			PRINT_INFO( "Found key [ %s ] with value [ %s ]", name.c_str(), ss.str().c_str() );
			return current;
		}
		return getMetaParamRec<__Desired>( name, rest... );
	}

	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamDefRec( const std::string & name, const __Desired & default_value, const std::string & current_name, __Current & current, __Rest&&... rest )
	{
		if( name == current_name )
		{
			std::stringstream ss;
			ss << current;
			PRINT_INFO( "Found key [ %s ] with value [ %s ]", name.c_str(), ss.str().c_str() );
			return current;
		}
		return getMetaParamDefRec<__Desired>( name, default_value, rest... );
	}

	// ####

	template<class __Desired, class... __Rest>
	static __Desired getMetaParam( const std::string & name, __Rest&&... rest )
	{
		// make sure desired type exists in list; otherwise fail at compile time
		getFirstOfType<__Desired>( rest... );
		return getMetaParamRec<__Desired>( name, rest... );
	}

	template<class __Desired, class... __Rest>
	static __Desired getMetaParamDef( const std::string & name, const __Desired & default_value, __Rest&&... rest )
	{
		return getMetaParamDefRec<__Desired>( name, default_value, rest... );
	}

	template<class __Type>
	struct ptr_types
	{
		typedef boost::shared_ptr<__Type> _Shared;
		typedef boost::shared_ptr<__Type const> _Const;
	};

	/*template<class __MessagePtr>
	typename std::enable_if<(boost::is_base_of<ros::Message, typename __MessagePtr::element_type>::value), typename __MessagePtr::element_type>::type
	getMessageType( const __MessagePtr & message_ptr )
	{
		return typename __MessagePtr::element_type();
	}*/

	template<class __Message, template<typename> class __Ptr>
	typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), __Message>::type
	getMessageType( const __Ptr<__Message const> & message )
	{
		return __Message();
	}

	template<class __Message, template<typename> class __Ptr>
	typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), __Message>::type
	getMessageType( const __Ptr<__Message> & message )
	{
		return __Message();
	}

	template<class __Message>
	typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), __Message>::type
	getMessageType( const __Message & message )
	{
		return message;
	}

	template<class __Message>
	typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), typename __Message::ConstPtr>::type
	make_const_shared( const __Message & message )
	{
		return typename __Message::ConstPtr( new __Message( message ) );
	}

	template<class __Message>
	typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), typename __Message::Ptr>::type
	make_shared( const __Message & message )
	{
		return typename __Message::Ptr( new __Message( message ) );
	}
}

#endif // QUICKDEVCPP_QUICKDEV_TYPEUTILS_H_
