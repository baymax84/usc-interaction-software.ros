#include <ros/ros.h>
#include <bandit_msgs/JointArray.h>

#include <bandit/bandit.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>

bandit::Bandit g_bandit;

// This callback is invoked when we get a new joint command
void jointIndCB(const bandit_msgs::JointConstPtr& j)
{
  // Set the joint position
  g_bandit.setJointPos(j->id, j->angle);

  // Push out positions to bandit
  g_bandit.sendJointPos();
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
    g_bandit.setJointPos(joint_iter->id, joint_iter->angle);
  }
  // Push out positions to bandit

  g_bandit.sendJointPos();
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
  nh.param("~/port", port, std::string("/dev/ttyUSB0"));

  ros::Publisher joint_pub = nh.advertise<bandit_msgs::JointArray>("joint_state", 0);

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
      if (g_bandit.getJointType(i) == bandit::SMART_SERVO)
        g_bandit.setJointPIDConfig(i, 100, 0, 0, -4000, 4001, 50, 0); 
    }

    // Synchronize PID gains and wait 5 seconds for confirmation.
    // (I don't like the fact that this is calling
    // processPendingMessages() behind the scenes.  That's my bad.
    // Ticket already submitted in trac.)
    if (g_bandit.syncAllPIDConfigs(5000000))
      ROS_INFO("All modules confirmed PID configuration.\n");
    else 
      ROS_ERROR("Some modules failed to configure\n");


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
    g_bandit.sendJointPos();


    // Now that things are supposeldy up and running, subscribe to
    // joint messages
    ros::Subscriber joint_sub = nh.subscribe("joint_cmd", 1, jointCB);
    ros::Subscriber ind_joint_sub = nh.subscribe("joint_ind", 1, jointIndCB);

    while (nh.ok())
    {
      // Process any pending messages from bandit
      if (!g_bandit.processPendingMessage(500000))
        ROS_WARN("Timed out waiting for messages from bandit\n");
      // Handle any ROS stuff we need to do
      ros::spinOnce();
    }

  } catch (bandit::BanditException& e)
  {
    ROS_FATAL("Caught bandit exception: %s\n", e.what());
  }

  return 0;
}
