#ifndef ROSCPP_SIMPLE_ROS_SHARED_MEMORY_PUBLISHER_H_
#define ROSCPP_SIMPLE_ROS_SHARED_MEMORY_PUBLISHER_H_

#include <ros/shared_memory_user.h>
#include <ros/serialization.h>
#include <boost/type_traits/is_base_of.hpp>
#include <ros/message.h>
#include <type_traits>

namespace ros
{

class SharedMemoryPublisher : public SharedMemoryUser
{

public:
	template<class... __Args>
	SharedMemoryPublisher( __Args&&... args )
	:
		SharedMemoryUser( args... )
	{
		//
	}

	template<class __Message>
	typename std::enable_if<(!boost::is_base_of<ros::Message, __Message>::value), void>::type
	publish( __Message * message, const unsigned int & size )
	{
		storage_.push( message, size );
	}

	template<class __Message>
	typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), void>::type
	publish( const __Message & message )
	{
		using namespace serialization;

		//ros::SerializedMessage m;
		//m.type_info = &typeid( __Message );
		//m.message = message;

		SerializedMessage serialized_message = serializeMessage<__Message>( message );

		storage_.push( serialized_message.message_start, serialized_message.num_bytes );
	}

	template<class __Message>
	typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), void>::type
	publish( const boost::shared_ptr<__Message>& message )
	{
		publish( *message );
	}
};

} // ros

#endif // ROSCPP_SIMPLE_ROS_SHARED_MEMORY_PUBLISHER_H_
