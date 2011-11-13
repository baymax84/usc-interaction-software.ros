#ifndef ROSCPP_SIMPLE_ROS_SHARED_MEMORY_SUBSCRIBER_H_
#define ROSCPP_SIMPLE_ROS_SHARED_MEMORY_SUBSCRIBER_H_

#include <ros/shared_memory_user.h>
#include <ros/serialization.h>
#include <boost/type_traits/is_base_of.hpp>
#include <ros/message.h>
#include <type_traits>

template<class __Data>
struct converter
{
	static void convert(){}
};

template<>
struct converter<std::string>
{
	static std::string convert( void* ptr )
	{
		return std::string( static_cast<char*>( ptr ) );
	}
};

namespace ros
{

class SharedMemorySubscriber : public SharedMemoryUser
{

public:
	template<class... __Args>
	SharedMemorySubscriber( __Args&&... args )
	:
		SharedMemoryUser( args... )
	{
		//
	}

	template<class __Data>
	typename std::enable_if<(!boost::is_base_of<ros::Message, __Data>::value), __Data>::type
	fetch() const
	{
		return converter<__Data>::convert( storage_.pull() );
	}

	template<class __Message>
	typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), __Message>::type
	fetch() const
	{
		using namespace serialization;

		ros::SerializedMessage m;

		__Message message;
		m.type_info = &typeid( __Message );
		m.num_bytes = message.serializationLength();
		m.message_start = static_cast<uint8_t*>( storage_.pull() );

		deserializeMessage<__Message>( m, message );

		return message;
	}
};

}// ros

#endif // ROSCPP_SIMPLE_ROS_SHARED_MEMORY_SUBSCRIBER_H_
