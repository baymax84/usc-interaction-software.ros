/***************************************************************************
 *  nodes/multi_subscriber.cpp
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
#include <quickdev/multi_subscriber.h>
#include <quickdev/param_reader.h>
#include <ros/rate.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

class MultiSubscriberNode
{
public:
    ros::MultiSubscriber<> multi_sub_;
    ros::Rate loop_rate_;

    MultiSubscriberNode( ros::NodeHandle & nh )
    :
        multi_sub_(),
        loop_rate_( 10 )
    {
        auto loop_rate = ros::ParamReader<double, 1>::readParam( nh, "loop_rate", 10 );
        loop_rate_ = ros::Rate( loop_rate );

        multi_sub_.addSubscriber(
                nh,
                "string",
                &MultiSubscriberNode::stringCB, this );

        multi_sub_.addSubscriber(
                nh,
                "point",
                &MultiSubscriberNode::pointCB, this );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( stringCB, std_msgs::String )
    {
        printf( "Got string: %s\n", msg->data.c_str() );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( pointCB, geometry_msgs::Point )
    {
        printf( "Got point: [%f %f %f]\n", msg->x, msg->y, msg->z );
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
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

QUICKDEV_INST_NODE( MultiSubscriberNode, "multi_subscriber" )
