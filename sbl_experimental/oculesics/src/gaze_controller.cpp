/*******************************************************************************
 *
 *      gaze_controller
 * 
 *      Copyright (c) 2011, edward
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "interaction-ros-pkg" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include <ros/ros.h>
#include <gaze_chain/core.h>
#include <geometry_msgs/Vector3.h>
#include <bandit_msgs/JointArray.h>

void operator<<( geometry_msgs::Vector3 & storage, geometry_msgs::Vector3 & input )
{
	storage = input;
}

template<class __JointArrayMessageType, class __AngleMessageType = geometry_msgs::Vector3, class __PositionMessageType = geometry_msgs::Vector3>
class GazeControllerBase
{
public:
	typedef GazeChain<double> _GazeChain;
	typedef __JointArrayMessageType _JointArrayMessageType;
	typedef __AngleMessageType _AngleMessageType;
	typedef __PositionMessageType _PositionMessageType;
	typedef _GazeChain::_AngleDataType _AngleDataType;

protected:
	ros::NodeHandle nh_priv;
	ros::Rate desired_frequency;

	_GazeChain gaze_chain;

	ros::Subscriber angle_sub, position_sub;
	ros::Publisher joint_array_pub;

	_AngleDataType desired_angle;

	_AngleMessageType angle_cache;
	_PositionMessageType position_cache;
	_JointArrayMessageType joint_array_message;

	std::string angle_topic_name, position_topic_name, joint_array_topic_name;

public:
	GazeControllerBase( ros::NodeHandle & nh, double desired_frequency_ = 10 ) :
		nh_priv( "~" ), desired_frequency( desired_frequency_ )
	{
		nh_priv.param( "angle_topic_name", angle_topic_name, std::string( "desired_angle" ) );
		nh_priv.param( "position_topic_name", position_topic_name, std::string( "desired_position" ) );
		nh_priv.param( "joint_array_topic_name", joint_array_topic_name, std::string( "joint_array" ) );

		angle_sub = nh_priv.subscribe( angle_topic_name, 1, &GazeControllerBase::angleCB__, this );
		position_sub = nh_priv.subscribe( position_topic_name, 1, &GazeControllerBase::positionCB__, this );
		joint_array_pub = nh_priv.advertise<_JointArrayMessageType> ( joint_array_topic_name, 1 );
	}

	/*_AngleMessageType::ConstPtr getAngleMessageFromPosition( const _PositionMessageType::ConstPtr & position_msg )
	{
		_AngleMessageType::Ptr angle_message = new _AngleMessageType;

		// do conversion

		_AngleMessageType::ConstPtr result = angle_message;
		return result;
	}*/

	void angleCB__( const typename _AngleMessageType::ConstPtr & angle_msg )
	{
		angle_cache = *angle_msg;
		desired_angle << *angle_msg;
		angleCB( angle_msg );
	}

	virtual void angleCB( const typename _AngleMessageType::ConstPtr & angle_msg )
	{

	}

	void positionCB__( const typename _PositionMessageType::ConstPtr & position_msg )
	{
		position_cache = *position_msg;
		positionCB( position_msg );

		//angleCB__( getAngleMessageFromPosition( position_msg) );
	}

	virtual void positionCB( const typename _PositionMessageType::ConstPtr & position_msg )
	{

	}

	virtual void updateController()
	{

	}

	void spin()
	{
		while ( ros::ok() )
		{
			updateController();
			joint_array_pub.publish( joint_array_message );
			desired_frequency.sleep();
		}
	}
};

template<class __JointArrayMessageType, class __AngleMessageType = geometry_msgs::Vector3, class __PositionMessageType = geometry_msgs::Vector3>
class GazeController : public GazeControllerBase<__JointArrayMessageType, __AngleMessageType, __PositionMessageType> {
public:
GazeController( ros::NodeHandle & nh ) : GazeControllerBase<__JointArrayMessageType, __AngleMessageType, __PositionMessageType>( nh )
{

}
};

// template specialization for bandit_msgs::JointArray and geometry_msgs::Vector3
template<>
class GazeController<bandit_msgs::JointArray, geometry_msgs::Vector3, geometry_msgs::Vector3> : public GazeControllerBase<bandit_msgs::JointArray, geometry_msgs::Vector3, geometry_msgs::Vector3>
{
public:
	GazeController( ros::NodeHandle & nh ) : GazeControllerBase<bandit_msgs::JointArray, geometry_msgs::Vector3, geometry_msgs::Vector3>( nh )
	{

	}

	void updateController()
	{
		_GazeChain::_JointArrayType joint_array = this->gaze_chain.update( this->desired_angle );

		for( unsigned int i = 0; i < joint_array.size(); ++i )
		{
			// the incoming joint array is going to be from the
			this->joint_array_message.joints[i] = joint_array[i];
		}
	}
};

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "gaze_controller" );
	ros::NodeHandle nh;

	GazeController<bandit_msgs::JointArray, geometry_msgs::Vector3, geometry_msgs::Vector3> gaze_controller( nh );
	gaze_controller.spin();

	return 0;
}
