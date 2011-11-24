#ifndef ROSCPP_SIMPLE_ROS_SHARED_MEMORY_STORAGE_H_
#define ROSCPP_SIMPLE_ROS_SHARED_MEMORY_STORAGE_H_

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <string>
#include <sstream>
#include <iostream>

namespace ros
{

class SharedMemoryStorage
{
private:
	typedef boost::interprocess::shared_memory_object _InternalStorage;
	typedef boost::interprocess::mapped_region _Storage;

	static int setStorageId( int inc = 0 )
	{
		static int storage_id = inc;
		storage_id += inc;
		return storage_id;
	}

	static int makeStorageId()
	{
		setStorageId();
		//static unsigned int storage_id = 0;
		return setStorageId( 1 );
	}

	int releaseStorageId()
	{
		return setStorageId( -1 );
	}

	static std::string makeName( unsigned int id )
	{
		std::stringstream ss;
		ss << "SharedMemoryStorage" << id;
		return ss.str();
	}

	static void removeStorage( const std::string & name )
	{
		_InternalStorage::remove( name.c_str() );
	}

	static _InternalStorage createStorage( const std::string & name )
	{
		removeStorage( name );
		return _InternalStorage( boost::interprocess::create_only, name.c_str(), boost::interprocess::read_write );
	}

	std::string name_;
	_InternalStorage internal_storage_;
	_Storage storage_;
	bool initialized_;
	bool is_ref_;

public:
	SharedMemoryStorage() : name_( makeName( makeStorageId() ) ), internal_storage_( createStorage( name_ ) ), initialized_( false ), is_ref_( false )
	{
		std::cout << "Creating new shared memory storage: " << name_ << std::endl;
	}

	SharedMemoryStorage( const std::string & name ) : name_( name ), internal_storage_( boost::interprocess::open_only, name_.c_str(), boost::interprocess::read_write ), storage_( internal_storage_, boost::interprocess::read_write ), initialized_( true ), is_ref_( true )
	{
		std::cout << "Connected to existing shared memory storage: " << name_ << std::endl;
	}

	const std::string & getName() const
	{
		return name_;
	}

	void initialize( unsigned int size )
	{
		if( initialized_ ) return;

		std::cout << "Initializing storage: " << size << " bytes" << std::endl;
		internal_storage_.truncate( size );
		storage_ = _Storage( internal_storage_, boost::interprocess::read_write );
		initialized_ = true;
	}

	~SharedMemoryStorage()
	{
		if( !is_ref_ )
		{
			std::cout << "Removing storage" << std::endl;
			removeStorage( name_ );
			releaseStorageId();
		}
	}

	template<class __Source>
	void push( __Source* source, unsigned int size )
	{
		if( !initialized_ )
		{
			std::cout << "Storage " << name_ << " not initialized; cannot push data" << std::endl;
			return;
		}
		std::memcpy( storage_.get_address(), source, size );
	}

	void* pull() const
	{
		if( !initialized_ )
		{
			std::cout << "Storage " << name_ << " not initialized; cannot pull data" << std::endl;
			return NULL;
		}
		return storage_.get_address();
	}
};

} // ros

#endif // ROSCPP_SIMPLE_ROS_SHARED_MEMORY_STORAGE_H_
