#include <time.h>
#include <sys/stat.h>

#include "ros/ros.h"
#include "ros/forwards.h"
#include "log_msgs/LogState.h"
#include "rosrecord/Recorder.h"
#include "rosrecord/AnyMsg.h"
#include "topic_tools/shape_shifter.h"

#include <boost/thread.hpp>

#include <set>
#include <queue>

class OutgoingMessage
{
public:
  OutgoingMessage(std::string _topic_name, topic_tools::ShapeShifter::ConstPtr _msg, ros::Time _time) :
    topic_name(_topic_name), msg(_msg), time(_time) {}

  std::string topic_name;
  topic_tools::ShapeShifter::ConstPtr msg;
  ros::Time time;

};


ros::record::Recorder recorder_;

std::vector<std::string> topics_;
std::set<std::string> currently_recording_;
int count_;
std::queue<OutgoingMessage> queue_;
boost::mutex queue_mutex_;
boost::condition_variable_any queue_condition_;
boost::thread* record_thread;

char fname_[1024];
std::string prefix_;
char tgt_fname_[2048];

int exit_code_;
bool check_master_;
bool running_;
ros::Subscriber log_sub_;

void add( std::string s )
{
}

//! Callback to be invoked to actually do the recording
void do_queue(topic_tools::ShapeShifter::ConstPtr msg,
               std::string topic_name,
               boost::shared_ptr<ros::Subscriber> subscriber,
               boost::shared_ptr<int> count)
{

  // Actually record the message to file
  //  g_recorder.record(topic_name, msg, ros::Time::now());
  OutgoingMessage out(topic_name, msg, ros::Time::now());

  {
    boost::mutex::scoped_lock lock(queue_mutex_);
    queue_.push(out);
  }
  queue_condition_.notify_all();

  // If we are book-keeping count, decrement and possibly shutdown
  if ((*count) > 0)
  {
    (*count)--;
    if ((*count) == 0)
      subscriber->shutdown();
  }
}

//! Thread that actually does writing to file.
// TODO: Do some checking here to make sure our queue isn't getting too big.
void do_record()
{
  ros::NodeHandle nh;

  // Technically the g_queue_mutex should be locked while checking empty
  // Except it should only get checked if the node is not ok, and thus
  // it shouldn't be in contention.
  //
  // change to add condition for stopped
  while ( running_ && (nh.ok() || !queue_.empty()) )
  {
    boost::unique_lock<boost::mutex> lock(queue_mutex_);
    while(queue_.empty())
    {
      if (!nh.ok() || !running_ )
      {
        return;
      }
      queue_condition_.wait(lock);
    }

    OutgoingMessage out = queue_.front();
    queue_.pop();

    lock.release()->unlock();

    recorder_.record(out.topic_name, out.msg, out.time);
  }
}


void do_check_master(const ros::TimerEvent& e, ros::NodeHandle& node_handle)
{
  ros::master::V_TopicInfo all_topics;

  if (ros::master::getTopics(all_topics))
  {
    // For each topic
    for (ros::master::V_TopicInfo::iterator topic_iter = all_topics.begin();
         topic_iter != all_topics.end();
         topic_iter++)
    {
      // If we're not currently recording it, do so
      if (currently_recording_.find(topic_iter->name) == currently_recording_.end())
      {
        boost::shared_ptr<int> count(new int(count_));
        boost::shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
        *sub = node_handle.subscribe<topic_tools::ShapeShifter>(topic_iter->name, 100, boost::bind(&do_queue, _1, topic_iter->name, sub, count));
        currently_recording_.insert(topic_iter->name);
      }
    }
  }
}

void do_check_subscribers_left(const ros::TimerEvent& e)
{
  ros::V_string subscribed_topics;
  ros::this_node::getSubscribedTopics(subscribed_topics);

  // If there is only 1 topic left (time), we can shutdown
  if (subscribed_topics.size() == 1)
    ros::shutdown();
}

void start()
{
  if( running_ ) return;
  ROS_INFO( "starting log...");

  // bag filename (repeat for each instance) 

  ros::NodeHandle node_handle;

  char name[1024];
  name[0] = 0;
  //prefix[0] = 0;
  time_t t = ::time(NULL);
  struct tm *tms = localtime(&t);
  snprintf(name, sizeof(name), "%s/%d-%02d-%02d-%02d-%02d-%02d-topic.bag",
           prefix_.c_str(),
           tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
           tms->tm_hour     , tms->tm_min  , tms->tm_sec);
  ROS_INFO( "filename: [%s]", name );
  strcpy(tgt_fname_, name);
  snprintf(fname_, sizeof(fname_), "%s.active", tgt_fname_);

  // start ros recorder (repeat for each instance) 
  //recorder_ = new ros::record::Recorder(n_.getNode() );
  if( recorder_.open(std::string(fname_)))
  {
    for( std::vector<std::string>::iterator i = topics_.begin(); i != topics_.end(); i++ )
    {
      boost::shared_ptr<int> count(new int(count_));
      boost::shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
      *sub = node_handle.subscribe<topic_tools::ShapeShifter>(*i, 100, boost::bind(&do_queue, _1, *i, sub, count));
    }
  } else {
    exit_code_ = 1;
    ros::shutdown();
  }

  ros::Timer check_master_timer;
  if (check_master_)
    check_master_timer = node_handle.createTimer(ros::Duration(1.0), boost::bind(&do_check_master, _1, boost::ref(node_handle)));

  ros::Timer check_subscribers_left_timer;
  if (count_ >= 0)
    check_subscribers_left_timer = node_handle.createTimer(ros::Duration(0.1), &do_check_subscribers_left);
  
  running_ = true;

  // Spin up a thread for actually writing to file
  record_thread = new boost::thread(&do_record);
  ROS_INFO( "started log...");
}


void stop()
{
  if( !running_ ) return;
  ROS_INFO( "stopping log...\n" );
  /* stop recording (repeat for each instance) */
  running_ = false;
  queue_condition_.notify_all();
  record_thread->join();
  delete record_thread;

  recorder_.close();
  rename( fname_,tgt_fname_);

  ROS_INFO( "stopped log...\n" );
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
  //ros::init( argc, argv, "log1");
  //std::map<std::string, std::string> remappings;
  ros::init( argc, argv, "log1");
  //std::string node_name = ros::this_node::getName() + "/";
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  check_master_ = false;
  running_ = false;
  /* topics (do once at startup) */

  std::string topic_string;
  nh_priv.param( "topics", topic_string, std::string("") );
  nh_priv.param( "prefix", prefix_, std::string("") );
  //n.param( "/b3ia/logger1/topics", topic_string, std::string("") );
  //n.param( "/b3ia/logger1/prefix", prefix_, std::string("") );
  ROS_INFO( "topics: %s(%s)", topic_string.c_str(),prefix_.c_str() );

  std::string::size_type i = 0;
  std::string::size_type j = topic_string.find(',');

  if( j == std::string::npos )
  {
    std::string ss = topic_string.substr(i,topic_string.length());
    topics_.push_back(ss);
  }

  while( j != std::string::npos )
  {
    std::string s = topic_string.substr(i,j-i);
    topics_.push_back(s);

    i = ++j;
    j = topic_string.find(',', j);
    if( j == std::string::npos )
    {
      std::string ss = topic_string.substr(i,topic_string.length());
      topics_.push_back(ss);
    }
  }

  log_sub_ = nh.subscribe( "logstate", 1000, log_cb);
  ros::MultiThreadedSpinner s(10);
  ros::spin(s);

  ROS_INFO( "b3ia_logger is done..." );
  stop();
  return 0;
}
