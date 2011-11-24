#ifndef ROSCPP_SIMPLE_ROS_SHARED_MEMORY_USER_H_
#define ROSCPP_SIMPLE_ROS_SHARED_MEMORY_USER_H_

#include <ros/shared_memory_storage.h>

namespace ros
{

class SharedMemoryUser
{
protected:
	SharedMemoryStorage storage_;

public:
	SharedMemoryUser( const unsigned int & buffer = 524288 )
	{
		storage_.initialize( buffer );
	}

	SharedMemoryUser( const SharedMemoryStorage & storage ) : storage_( storage.getName() )
	{
		//std::cout << "Connected to storage with name: " << storage_.getName() << std::endl;
	}

	SharedMemoryUser( const std::string & name ) : storage_( name )
	{
		//std::cout << "Connected to storage with name: " << storage_.getName() << std::endl;
	}

	const SharedMemoryStorage & getStorage() const
	{
		return storage_;
	}
};

} // ros

#endif // ROSCPP_SIMPLE_ROS_SHARED_MEMORY_USER_H_
