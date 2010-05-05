/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010. David Feil-Seifer
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dcam1394/OSTFileReader.h>
#include <deque>

class CameraMsgs  {
  public:
    sensor_msgs::CameraInfo info;
    sensor_msgs::Image      img;
};

std::deque<CameraMsgs> left_queue;
std::deque<CameraMsgs> right_queue;

void left_cb( const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& infomsg )
{
  CameraMsgs msgs;
  msgs.info = *infomsg;
  msgs.img = *msg;

  left_queue.push_back(msgs);
}

void right_cb( const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& infomsg )
{
  CameraMsgs msgs;
  msgs.info = *infomsg;
  msgs.img = *msg;

  right_queue.push_back(msgs);
}
int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "stereo_sync" );
  ros::NodeHandle nh;
  double tolerance;
  ros::Rate loop_rate( 100 );
  nh.param("tolerance", tolerance, 0.05 );
  ros::Duration msg_tol(tolerance);
  image_transport::ImageTransport it(nh);
  image_transport::CameraSubscriber left_sub = it.subscribeCamera("left/image", 1, left_cb );
  image_transport::CameraSubscriber right_sub = it.subscribeCamera("right/image", 1, right_cb );
  image_transport::CameraPublisher left_pub = it.advertiseCamera("stereo/left/image_raw",1);
  image_transport::CameraPublisher right_pub = it.advertiseCamera("stereo/right/image_raw",1);

  std::string ostfilename;
  std::vector<sensor_msgs::CameraInfo> file_info;
  nh.param( "/calibration_file", ostfilename, std::string("") );
  if( ostfilename != "" )
  {
    ROS_INFO ("parsing calibration file: %s", ostfilename.c_str() );
    file_info = parseOST( ostfilename );
    ROS_INFO( "parsed: %d", file_info.size() );
  }

  while( ros::ok() )
  {
    while( left_queue.size() >= 1 && right_queue.size() >= 1 )
    {
      // is there a match between the front two elements?
      if( fabs((left_queue.front().info.header.stamp-right_queue.front().info.header.stamp).toSec())<tolerance)
      {
        CameraMsgs left_msg = left_queue.front();
        CameraMsgs right_msg = right_queue.front();

        if( file_info.size() == 2 )
        {
          left_msg.info.P = file_info[0].P;
          left_msg.info.D = file_info[0].D;
          left_msg.info.R = file_info[0].R;
          left_msg.info.K = file_info[0].K;
          right_msg.info.P = file_info[1].P;
          right_msg.info.D = file_info[1].D;
          right_msg.info.R = file_info[1].R;
          right_msg.info.K = file_info[1].K;
          ROS_INFO( "adjusting calibration" );
       }

        // publish
        left_pub.publish(left_msg.img, left_msg.info, left_msg.info.header.stamp);
        right_pub.publish(right_msg.img, right_msg.info, left_msg.info.header.stamp);
        left_queue.pop_front();
        right_queue.pop_front();
      }
      else if( left_queue.front().info.header.stamp-right_queue.front().info.header.stamp > msg_tol )
      {
        // right msg too old
        right_queue.pop_front();
      }
      else if( right_queue.front().info.header.stamp-left_queue.front().info.header.stamp > msg_tol )
      {
        //left msg too old
        left_queue.pop_front();
      }
      
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

