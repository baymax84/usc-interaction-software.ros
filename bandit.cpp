#include <ros/ros.h>
#include <bandit_msgs/JointArray.h>

#include <bandit/bandit.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>

bandit::Bandit g_bandit;

#define DTOR( a ) a * M_PI / 180.0
#define RTOD( a ) a * 180.0 / M_PI

/*
int joints[19];

// Tells a joint to move in the same or opposite direction of the normal Bandit movement
int direction[19];
// Home positions of the joints for Bandit
double home[19];
*/

// This callback is invoked when we get a new joint command
void jointIndCB(const bandit_msgs::JointConstPtr& j)
{
  // Set the joint position
  int newid = j->id;
  printf( "j->angle: %f\n", j->angle );
  // scale to home
  double pos = j->angle;
  //double pos = DTOR(direction[j->id]*(RTOD(j->angle)+home[j->id]));

  g_bandit.setJointPos(newid, pos );
  printf( "setting [%d] to [%0.2f]\n", newid, RTOD(pos) );
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
    double pos = joint_iter->angle;
    // Set the joint position
    g_bandit.setJointPos(joint_iter->id, pos);
  }
  // Push out positions to bandit

  g_bandit.sendAllJointPos();
}

// This callback is invoked when we get new state from bandit
void stateCB(ros::Publisher& joint_pub)
{
  bandit_msgs::JointArray j;
  bandit_msgs::Joint joint;

  // For every joint
  for (int i = 0; i < 19; i++)
  {
    // Set the id and angle
    joint.id = i;
    joint.angle = g_bandit.getJointPos(i);

    // Add to array
    j.joints.push_back(joint);
  }

  // Publish to other nodes
  joint_pub.publish(j);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bandit");
  ros::NodeHandle nh;

  // Retrieve port from parameter server
  std::string port;
  nh.param("~/port", port, std::string("/dev/ttyS1"));

  ros::Publisher joint_pub = nh.advertise<bandit_msgs::JointArray>("joint_state", 0);
/*
    std::string homestring, dirsstring;

    nh.param( "~home", homestring, std::string("0,0,-54,-84,-117,51,0,65,88,62,74,121,-57,0,0,0,0,0,0"));
    nh.param( "~dirs", dirsstring, std::string("-1,1,1,-1,-1,-1,-1,1,1,-1,-1,-1,1,-1,1,1,1,1,1"));

    for( int i = 0; i < 19; i++ )
    {
      direction[i] = 1;
      home[i] = 0;
    }

    std::string::size_type i = 0;
    std::string::size_type j = homestring.find(',');

    int ii = 0;
    printf( "homes: \n\n" );
    while( j != std::string::npos )
    {
      std::string s = homestring.substr(i,j-i);
      home[ii++] = atoi(s.c_str() );
      printf( "%s:%f\n", s.c_str(), home[ii-1] );
      i= ++j;
      j = homestring.find(',',j);
    }

    printf( "\n" );

    i = 0;
    j = dirsstring.find(',');
    ii = 0;
    printf( "dirss: \n\n" );
    while( j != std::string::npos )
    {
      std::string s = dirsstring.substr(i,j-i);
      direction[ii++] = atoi(s.c_str() );
      printf( "%s:%d\n", s.c_str(), direction[ii-1] );
      i= ++j;
      j = dirsstring.find(',',j);
    }
    printf( "\n" );
   // this maps the joints from the original driver to what's used in the gazebo model
  joints[0]  = 12;
  joints[1]  = 5;
  joints[2]  = 10;
  joints[3]  = 9;
  joints[4]  = 8;
  joints[5]  = 6;
  joints[6]  = 7;
  joints[7]  = 7; // left wrist bend - does nothing right now
  joints[8]  = 7; // left hand open/close - does nothing right now
  joints[9]  = 4;
  joints[10] = 3;
  joints[11] = 2;
  joints[12] = 0;
  joints[13] = 1;
  joints[14] = 1; // right wrist bend - does nothing right now
  joints[15] = 1; // right hand open/close - does nothing right now
*/

  try 
  {
    // This callback gets called whenever processPendingMessages
    // receives a valid state update from bandit
    g_bandit.registerStateCB(boost::bind(&stateCB, boost::ref(joint_pub)));
    
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
    g_bandit.setJointPos(0,  0.0); //"head pitch",        
    g_bandit.setJointPos(1,  0.0); //"head pan",          
    g_bandit.setJointPos(2,  0.0); //"left shoulder F/B", 
    g_bandit.setJointPos(3,  0.0); //"left shoulder I/O", 
    g_bandit.setJointPos(4,  0.0); //"left elbow twist",  
    g_bandit.setJointPos(5,  0.0); //"left elbow",        
    g_bandit.setJointPos(6,  0.0); //"left wrist twist",  
    g_bandit.setJointPos(7,  0.5);  //"left wrist tilt",   
    g_bandit.setJointPos(8,  0.5);  //"left hand grab",    
    g_bandit.setJointPos(9,  0.0); //"right shoulder F/B",
    g_bandit.setJointPos(10, 0.0); // "right shoulder I/O"
    g_bandit.setJointPos(11, 0.0); // "right elbow twist",
    g_bandit.setJointPos(12, 0.0); // "right elbow",      
    g_bandit.setJointPos(13, 0.0); // "right wrist twist",
    g_bandit.setJointPos(14, 0.5);  // "right wrist tilt", 
    g_bandit.setJointPos(15, 0.5);  // "right hand grab",  
    g_bandit.setJointPos(16, 0.5);  // "eyebrows",         
    g_bandit.setJointPos(17, 0.5);  // "mouth top",        
    g_bandit.setJointPos(18, 0.5);  // "mouth bottom", 

    // Send bandit position commands:
    g_bandit.sendAllJointPos();


    // Now that things are supposeldy up and running, subscribe to
    // joint messages
    ros::Subscriber joint_sub = nh.subscribe("joint_cmd", 1, jointCB);
    ros::Subscriber ind_joint_sub = nh.subscribe("joint_ind", 1, jointIndCB);

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
