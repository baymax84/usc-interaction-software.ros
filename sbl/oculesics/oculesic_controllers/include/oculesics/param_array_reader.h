#include <ros/ros.h>
#include <type_traits>
#include <vector>
#include <sstream>

// a class that will read up to __Dim__ paramters with the name format "prefix_|n|postfix_"
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
	
	ParamArrayReader( ros::NodeHandle & nh, std::string prefix, std::string postfix = "", unsigned int start_index = 1 ) : nh_( nh ), prefix_( prefix ), postfix_( postfix ), start_index_( start_index )
	{
		readParams();
	}
	
	_Array & readParams()
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
