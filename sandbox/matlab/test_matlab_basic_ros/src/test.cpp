#include "mex.h"
#include <iostream>
#include <vector>
#include <ros/shared_memory_publisher.h>
#include <ros/shared_memory_subscriber.h>
#include <std_msgs/String.h>
#include <test_matlab/Vector3.h>
#include <matlab/mat.h>

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
	// We use a SharedMemoryPublisher to push data to another process
	// Here, we're allocating a new block of memory
	ros::SharedMemoryPublisher pub;

	// We use a SharedMemorySubscriber to pull data from another process
	// Here, we're connecting to the block of memory stored in pub
	ros::SharedMemorySubscriber sub( pub.getStorage().getName() );

	// The message to publish to our target process
	test_matlab::Vector3 v;
	v.x = 5;
	v.y = 6;
	v.z = 7;

	// Push the data into the shared memory block
	pub.publish( v );

	// The target process is designed to create its own SharedMemoryPublisher and SharedMemorySubscriber
	// Here, we pass the name of our publisher's storage (which is guaranteed to be unique) as the only argument to our target process
	// The target process will then connect to our publisher's storage, read the data from it, then publish some data back for us to read
	std::string command = "rosrun test_matlab_ros test_ros " + pub.getStorage().getName();
	std::system( command.c_str() );

	// Fetch the stored data as a test_matlab::Vector3
	// Technically we can fetch the data as any data type but doing so will likely fail under most other circumstances
	v = sub.fetch<test_matlab::Vector3>();

	// C++ wrapper around mxArray; create a 1x3 array of doubles
	// A new mxArray is automatically allocated (in this case, using mxCreateNumericMatrix( 1, 3, mxDOUBLE_CLASS, mxREAL ) )
	matlab::Mat<double> result( 1, 3 );

	// We can index the mat as expected
	result[0] = v.x;
	result[1] = v.y;
	result[2] = v.z;

	// We can easily print out result using an std::ostream
	// Alternatively, we can do: result.toString(); which will print the dimensions of the matrix and its data (formatted and structured)
	// To print just the data, we can do: result.dataToString();
	// If result was a matlab::Mat<char>, the call to result.datToString() would return the union of all the character elements rather than formatted data
	std::cout << "result: " << result << std::endl;

	// An implicit cast operator is defined inside matlab::MatlabMat which allows us to cleanly assign "result" to the first left-hand-side value
	// Alternatively, we can do: plhs[0] = result.getMat()
	plhs[0] = result;
}
