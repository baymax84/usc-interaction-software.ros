#include <ros/ros.h>
#include <bandit_msgs/JointArray.h>
#include <bandit_msgs/Params.h>
#include <sensor_msgs/JointState.h>
#include <bandit/bandit.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>

bandit::Bandit g_bandit;

#define DTOR( a ) a * M_PI / 180.0
#define RTOD( a ) a * 180.0 / M_PI



bandit_msgs::Params::Response param_res;

bool param(bandit_msgs::Params::Request    &req,
           bandit_msgs::Params::Response   &res )
{
  res = param_res;
  return true;
}


// This callback is invoked when we get a new joint command
void jointIndCB(const bandit_msgs::JointConstPtr& j)
{
  // Set the joint position
  ROS_INFO( "j->angle: %f\n", RTOD(j->angle) );

  // scale to home
  //double dpos = direction[j->id]*RTOD(j->angle)+home[j->id];
  //double pos = DTOR( dpos );

    ROS_INFO( "setting joint: %d to angle: %0.2f", j->id, RTOD(j->angle) );
  g_bandit.setJointPos(j->id, j->angle);

  // Push out positions to bandit
  g_bandit.sendAllJointPos();
}

// This callback is invoked when we get a new joint command
void jointCB(const bandit_msgs::JointArrayConstPtr& j)
{
  // Iterate through all joints in Joint Array
  for (std::vector<bandit_msgs::Joint>::const_iterator joint_iter = j->joints.begin();
       joint_iter != j->joints.end();
       joint_iter++)
  {
    // Set the joint position
    ROS_INFO( "j->angle: %f\n", RTOD(joint_iter->angle) );

    // scale to home
    //double dpos = direction[joint_iter->id]*RTOD(joint_iter->angle)+home[joint_iter->id];
    //double pos = DTOR( dpos );

    // Set the joint position
    ROS_INFO( "setting joint: %d to angle: %0.2f", joint_iter->id, joint_iter->angle );
    g_bandit.setJointPos(joint_iter->id, joint_iter->angle);
  }
  // Push out positions to bandit

  g_bandit.sendAllJointPos();


  printf( "=====================\n");
}

void targetCB(const sensor_msgs::JointStateConstPtr &t )
{
  for( int i = 0; i < t->name.size(); i++ )
  {
    for( int j = 0; j < 19; j++ )
    {
      if( g_bandit.getJointRosName(j) == t->name[i] )
      {
        g_bandit.setJointPos(j, t->position[i] );
      }
    }
  }

  g_bandit.sendAllJointPos();
}

// This callback is invoked when we get new state from bandit
void stateCB(ros::Publisher& joint_pub)
{
  sensor_msgs::JointState js;

  js.header.stamp = ros::Time::now();
  js.header.frame_id = "bandit_torso_link";

  // For every joint
  for (int i = 0; i < 19; i++)
  {
    if( g_bandit.getJointRosName(i) == std::string("eyebrows_joint") )
    {
      js.name.push_back(std::string("bandit_head_left_brow_joint"));
      js.position.push_back(-3*g_bandit.getJointPos(i));
      js.velocity.push_back(0);
      js.effort.push_back(0);
      js.name.push_back(std::string("bandit_head_right_brow_joint"));
      js.position.push_back(-3*g_bandit.getJointPos(i));
      js.velocity.push_back(0);
      js.effort.push_back(0);
    
    }
    else
    {
      js.name.push_back(g_bandit.getJointRosName(i));
      if( i > 16 )
        js.position.push_back(2*g_bandit.getJointPos(i));
      else 
        js.position.push_back(g_bandit.getJointPos(i));

      js.velocity.push_back(0);
      js.effort.push_back(0);
   }
  }

  
  ROS_INFO( "publishing..." );

  // Publish to other nodes
  joint_pub.publish(js);
}

int main(int argc, char** argv)
{
  // Tells a joint to move in the same or opposite direction
  // of the normal Bandit movement
  int direction[19];
  
  // Home positions of the joints for Bandit
  double home[19];

  ros::init(argc, argv, "bandit");
  ros::NodeHandle nh;

  // Retrieve port from parameter server
  std::string port;
  nh.param("port", port, std::string("/dev/ttyUSB0"));

  ros::Publisher joints_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
  //ros::Publisher joint_pub = nh.advertise<bandit_msgs::JointArray>("joint_state", 1000);

    std::string homestring, dirsstring;

    // joint offset parameters for Bandit #1 (RAM-2010Apr30)
    nh.param( "home", homestring, std::string("-3,10,-62,-74,-28,62,-11,0.5,0.6,52,80,26,-58,12,0.5,0.5,0.2,0.25,0.25,"));
    nh.param( "direction", dirsstring, std::string("-1,-1,1,1,1,-1,1,1,1,-1,-1,-1,1,-1,-1,1,1,1,-1,"));

    for( int i = 0; i < 19; i++ )
    {
      direction[i] = 1;
      home[i] = 0;
    }

    std::string::size_type i = 0;
    std::string::size_type j = homestring.find(',');

    int ii = 0;
    printf( "home: \n\n" );
    while( j != std::string::npos )
    {
      std::string s = homestring.substr(i,j-i);
      home[ii] = atof(s.c_str());
      printf( "%d:%s:%f\n", ii, s.c_str(), home[ii] );
      ++ii;
      i = ++j;
      j = homestring.find(',',j);
    }

    printf( "\n" );
    
    i = 0;
    j = dirsstring.find(',');
    ii = 0;
    printf( "direction: \n\n" );
    while( j != std::string::npos )
    {
      std::string s = dirsstring.substr(i,j-i);
      direction[ii] = atoi(s.c_str());
      printf( "%d:%s:%d\n", ii, s.c_str(), direction[ii] );
      ++ii;
      i= ++j;
      j = dirsstring.find(',',j);
    }
    printf( "\n" );
    
  try 
  {
    // This callback gets called whenever processPendingMessages
    // receives a valid state update from bandit
    g_bandit.registerStateCB(boost::bind(&stateCB, boost::ref(joints_pub)));
    
    ROS_INFO( "connecting to bandit on port: [%s]\n", port.c_str() );

    // Open the port
    g_bandit.openPort(port.c_str());

    // This 19 shouldn't be hard coded.  Will improve API for this
    // later.  Also, should set this up as a Service API to tweak
    // gains online as well.
    for (int i = 0; i < 19; i++)
    {
      // These default PID gains and offsets should be read from a config file
      if (g_bandit.getJointType(i) == smartservo::SMART_SERVO)
        g_bandit.setJointPIDConfig(i, 100, 0, 0, -4000, 4001, 50, 0);

      //set joint offsets and directions
      g_bandit.setJointDirection(i, direction[i]);
      //g_bandit.setJointOffset(i, DTOR(home[i]));
      if (g_bandit.getJointType(i) == smartservo::SMART_SERVO)
        g_bandit.setJointOffset(i, DTOR(home[i]));
      else
        g_bandit.setJointOffset(i, home[i]);
      
      //populate service response message
      param_res.id.push_back(i);
      param_res.name.push_back(g_bandit.getJointRosName(i));
      //param_res.min.push_back(RTOD(g_bandit.getJointMin(i)));
      //param_res.max.push_back(RTOD(g_bandit.getJointMax(i)));
      //param_res.pos.push_back(RTOD(g_bandit.getJointPos(i)));
      if (g_bandit.getJointType(i) == smartservo::SMART_SERVO)
      {
        param_res.min.push_back(RTOD(g_bandit.getJointMin(i)));
        param_res.max.push_back(RTOD(g_bandit.getJointMax(i)));
        param_res.pos.push_back(RTOD(g_bandit.getJointPos(i)));
      }
      else
      {
        param_res.min.push_back(RTOD(g_bandit.getJointMin(i)));
        param_res.max.push_back(RTOD(g_bandit.getJointMax(i)));
        param_res.pos.push_back(RTOD(g_bandit.getJointPos(i)));
      }
    }

    // Synchronize PID gains
    // we set the first time back in time to guarantee it gets hit
    ros::Time try_again = ros::Time::now() - ros::Duration(.1);
    int tries = 0;
    do
    {
      if (try_again < ros::Time::now())
      {
        g_bandit.sendAllPIDConfigs();        
        try_again = try_again + ros::Duration(1.0);
        if (++tries % 10 == 0)
          ROS_ERROR("Failed to configure PID settings after %d tries", tries);
      }
      
      g_bandit.processIO(10000);
      g_bandit.processPackets();

    }  while (!g_bandit.checkAllPIDConfigs());

    ROS_INFO("All PID settings configured successfully");

    // Push out initial state
    for( int i = 0; i < 19; i++ )
    {
      g_bandit.setJointPos(i,  0.0f);
    }

    // Send bandit position commands:
    g_bandit.sendAllJointPos();


    // Now that things are supposeldy up and running, subscribe to
    // joint messages
    ros::Subscriber joint_sub = nh.subscribe("joint_cmd", 1, jointCB);
    ros::Subscriber ind_joint_sub = nh.subscribe("joint_ind", 1, jointIndCB);
    ros::Subscriber target_sub = nh.subscribe("target_state", 1, targetCB );

    ros::ServiceServer service = nh.advertiseService("params", param);
    while (nh.ok())
    {
      // Process any pending messages from bandit
      g_bandit.processIO(10000);
      g_bandit.processPackets();
      ros::spinOnce();
    }

  } catch (bandit::BanditException& e)
  {
    ROS_FATAL("Caught bandit exception: %s\n", e.what());
  }

  return 0;
}
