/***************************************************************************
 *  nodes/multi_publisher.cpp
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

#include <quickdev/macros.h>
#include <quickdev/multi_publisher.h>
#include <ros/rate.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

class MultiPublisherNode
{
public:
	ros::MultiPublisher<> multi_pub_;
	ros::Rate loop_rate_;

	MultiPublisherNode( ros::NodeHandle & nh )
	:
		multi_pub_(),
		loop_rate_( 10 )
	{
		multi_pub_.addPublishers<
			std_msgs::String,
			geometry_msgs::Point>(
				nh,
				{ "string", "point" } );

		multi_pub_.addPublishers<
			std_msgs::String,
			geometry_msgs::Point>(
				nh,
				{ "string2", "point2" } );
	}

	QUICKDEV_SPIN_ONCE
	{
		std_msgs::String string_msg;
		string_msg.data = "hello";

		geometry_msgs::Point point_msg;
		point_msg.x = 1;
		point_msg.y = 2;
		point_msg.z = 3;

		// it is possible to publish 1 or more key-value pairs with a
		// single call to publish( ... )
		multi_pub_.publish(
			"string", string_msg,
			"point", point_msg );

		string_msg.data = "goodbye";

		multi_pub_.publish( "string2", string_msg );

		point_msg.z = 5;

		multi_pub_.publish( "point2", point_msg );

		string_msg.data = "whatever";
		point_msg = geometry_msgs::Point();

		multi_pub_.publish(
			"string", string_msg,
			"point", point_msg,
			"string2", string_msg,
			"point2", point_msg );
	}

	void spin()
	{
		while( ros::ok() )
		{
			spinOnce();
			ros::spinOnce();
			loop_rate_.sleep();
		}
	}
};

QUICKDEV_INST_NODE( MultiPublisherNode, "multi_publisher" )
