#include <ros/ros.h>
#include <bandit_msgs/JointArray.h>

#include <bandit/bandit.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>

bandit::Bandit g_bandit;

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
  nh.param("~/port", port, std::string("/dev/ttyS0"));

  ros::Publisher joint_pub = nh.advertise<bandit_msgs::JointArray>("joint_state", 0);

  try 
  {
    // This callback gets called whenever processPendingMessages
    // receives a valid state update from bandit
    g_bandit.registerStateCB(boost::bind(&stateCB, boost::ref(joint_pub)));
    
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


    // Now that things are supposeldy up and running, subscribe to
    // joint messages
    ros::Subscriber joint_sub = nh.subscribe("joint_cmd", 0, jointCB);

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
