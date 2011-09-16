#include <ros/ros.h>
#include <type_traits>
#include <vector>
#include <sstream>

#ifndef USC_ROS_TOOLS_ROS_TOOLS_PARAM_ARRAY_READER_H_
#define USC_ROS_TOOLS_ROS_TOOLS_PARAM_ARRAY_READER_H_

// a class that will read up to __Dim__ paramters with the name format "prefix_|n|postfix_"
// a dim of 0 means "load all matching params that are found, without limit"
// so you can do something like:
// - ParamArrayReader param_array_reader1( nh, ... ).readParams();
// - ParamArrayReader param_array_reader2( nh ).readParams( ... );
// - ParamArrayReader param_array_reader3( nh );
//   param_array_reader1.readParams( ... )
//   param_array_reader2.readParams( ... )
//   param_array_reader3.readParams( ... )

template<class __Storage, unsigned int __Dim__ = 0>
class ParamArrayReader
{
public:
	typedef __Storage _Storage;
	typedef std::vector<__Storage> _Array;
	
	ros::NodeHandle nh_;
	std::string prefix_;
	std::string postfix_;
	unsigned int start_index_;
	_Array params_;
	
	ParamArrayReader(){}
	
	ParamArrayReader( ros::NodeHandle & nh, const std::string & prefix, const std::string & postfix = "", unsigned int start_index = 1 ) : nh_( nh ), prefix_( prefix ), postfix_( postfix ), start_index_( start_index )
	{
		//
	}
	
public:
	_Array readParams()
	{
		return readParams( prefix_, postfix_, start_index_ );
	}
	
	_Array readParams( const std::string & prefix, const std::string & postfix = "", unsigned int start_index = 1 )
	{
		params_.clear();
		bool new_param_found = true;
	    unsigned int n = start_index_;
	    do
	    {
			std::stringstream ss;
			ss << prefix_ << n << postfix_;
			
			__Storage param_value;
			const std::string param_name = ss.str();
			
			if( nh_.getParam( param_name.c_str(), param_value ) )
			{
				std::cout << "Loaded param [" << param_name.c_str() << "] with value " << param_value << std::endl;
				params_.push_back( param_value );
				++n;
			}
			else new_param_found = false;
	    }
	    while( new_param_found && ( __Dim__ == 0 || n < __Dim__ + start_index_ ) );
		return params_;
	}
};

// a ParamArrayReader of length one can be initialized with a single
// nodehandle and readParam() can be called repeatedly to fetch params
// so you can do something like:
// - ParamArrayReader param_array_reader1( nodehandle, ... ).getParam();
// - ParamArrayReader param_array_reader2( nodehandle ).getParam( ... );
// - ParamArrayReader param_array_reader3( nodehandle );
//   param_array_reader3.getParam( "param1" );
//   param_array_reader2.getParam( "param2" );
//   param_array_reader1.getParam( "param3" );

template<class __Storage>
class ParamArrayReader<__Storage, 1>
{
public:
	ros::NodeHandle nh_;
	
	std::string param_name_;
	__Storage default_value_;
	
	ParamArrayReader(){}
	
	ParamArrayReader( ros::NodeHandle & nh, const std::string & param_name = "", const __Storage & default_value = __Storage() ) : nh_( nh )
	{
		//
	}
	
	__Storage readParam()
	{
		return readParam( param_name_, default_value_ );
	}
	
	__Storage readParam( const std::string & param_name, const __Storage & default_value = __Storage() )
	{
		__Storage param_value( default_value );
		if( !nh_.getParam( param_name.c_str(), param_value ) )
			ROS_WARN( "Could not find param %s under the given nodehandle", param_name );
		return param_value;
	}
};

#endif // USC_ROS_TOOLS_ROS_TOOLS_PARAM_ARRAY_READER_H_
