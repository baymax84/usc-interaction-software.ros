#include <time.h>
#include <sys/stat.h>

#include "ros/ros.h"
#include "ros/forwards.h"
#include "log_msgs/LogState.h"

#include "logger/recorder.h"
#include "rosbag/exceptions.h"
#include "topic_tools/shape_shifter.h"

#include "boost/program_options.hpp"

std::string prefix_;
bool running_;
bool shutting_down_;
ros::Subscriber log_sub_;
logger::Recorder *recorder_;
logger::RecorderOptions opts_;
ros::NodeHandle *nh_;

void start()
{
  if( running_ ) return;

	while( shutting_down_ && ros::ok() )
	{
		ros::Duration(0.25).sleep();
	}
	logger::RecorderOptions opts = opts_;

	recorder_ = new logger::Recorder(opts, *nh_);
	recorder_->run();
	running_ = true;
	ROS_INFO( "started log...");
}

void stop()
{
  if( !running_ ) return;
	shutting_down_ = true;
	recorder_->stop();
  running_ = false;
	//delete recorder_;
	shutting_down_ = false;
  ROS_INFO( "stopped log..." );
}

void log_cb( const log_msgs::LogStateConstPtr &msg )
{
  if( msg->state == 1 )
  {
    stop();
  }
  else if( msg->state == 0 )
  {
    start();
  }
}


int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "log1");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

	shutting_down_ = false;

	nh_ = &nh;

	std::string topic_string;

  nh_priv.param( "topics", topic_string, std::string("") );
  nh_priv.param( "prefix", prefix_, std::string("") );
  ROS_INFO( "topics: %s(%s)", topic_string.c_str(),prefix_.c_str() );

  std::string::size_type i = 0;
  std::string::size_type j = topic_string.find(',');

  if( j == std::string::npos )
  {
    std::string ss = topic_string.substr(i,topic_string.length());
    opts_.topics.push_back(ss);
  }

  while( j != std::string::npos )
  {
    std::string s = topic_string.substr(i,j-i);
    opts_.topics.push_back(s);

    i = ++j;
    j = topic_string.find(',', j);
    if( j == std::string::npos )
    {
      std::string ss = topic_string.substr(i,topic_string.length());
      opts_.topics.push_back(ss);
    }
  }

	opts_.record_all = false;
	opts_.regex = false;
	opts_.do_exclude = false;

	opts_.prefix = prefix_;
	opts_.append_date = true;

	opts_.snapshot = false;

  log_sub_ = nh.subscribe( "logstate", 1000, log_cb);
	//ros::spin();
  ros::MultiThreadedSpinner s(10);
  ros::spin(s);

  ROS_INFO( "log node is closing shutting down active logger..." );
  stop();
  return 0;
}

