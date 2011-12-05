/***************************************************************************
 *  nodes/message_array_cache.cpp
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
#include <quickdev/type_utils.h>
#include <quickdev/message_array_cache.h>
#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/param_reader.h>

#include <ros/rate.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <quickdev_examples/StringArray.h>

class MessageArrayCacheNode
{
public:
    typedef std_msgs::String _StringMsg;
    typedef quickdev_examples::StringArray _StringArrayMsg;
    typedef quickdev::StampedMessage<_StringMsg> _StampedStringMsg;
    typedef geometry_msgs::Point _PointMsg;

    ros::MultiSubscriber<> multi_sub_;
    ros::MultiPublisher<> multi_pub_;
    quickdev::TimedMessageArrayCache<quickdev::StampedMessage<_StringMsg>, quickdev::TimedMessageArrayCacheFlags::STAMPED_ON_UPDATE> timed_cache_;
    ros::Rate loop_rate_;

    MessageArrayCacheNode( ros::NodeHandle & nh )
    :
        loop_rate_( 10 )
    {
        auto loop_rate = ros::ParamReader<double, 1>::readParam( nh, "loop_rate", 10 );
        loop_rate_ = ros::Rate( loop_rate );

        multi_sub_.addSubscriber(
                nh,
                "string",
                &MessageArrayCacheNode::stringCB, this );

        multi_pub_.addPublishers<_StringMsg>( nh, { "string_array" } );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( stringCB, _StringMsg )
    {
        timed_cache_.updateMessage( _StampedStringMsg( *msg, ros::Time::now() ) );
    }

    QUICKDEV_SPIN_ONCE()
    {
        const auto now = ros::Time::now();

        timed_cache_.eraseOld( 2.0 );
        // get messages ready for output
        const auto messages = timed_cache_.getMessages();
        // get read-only messages (faster)
        // const auto messages = timed_cache_.getRawMessages();

        printf( "Cache contains:\n" );
        for( auto message = messages.begin(); message != messages.end(); ++message )
        {
            printf( "%s\n", message->data.c_str() );
        }
        printf( "----------\n" );

        _StringArrayMsg string_array;
        string_array.strings = messages;
        multi_pub_.publish( "string_array", string_array );
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

// instantiate our node; this macro expands to an int main( ... ) in which an instance of our node is created and started
//
QUICKDEV_INST_NODE( MessageArrayCacheNode, "message_array_cache" )
