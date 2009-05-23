#include <ros/ros.h>
#include <bandit_msgs/JointArray.h>

#include <bandit/bandit.h>

bandit::Bandit g_bandit;

void jointCB(const bandit_msgs::JointArray& j)
{
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bandit");
  ros::NodeHandle nh;

  std::string port;
  nh.param("~/port", port, std::string("/dev/ttyS0"));

  try 
  {

    g_bandit.openPort(port.c_str());

    // This 19 shouldn't be hard coded.  Will improve API for this later.
    // Also, should set this up as a Service API as well.
    for (int i = 0; i < 19; i++)
    {
      if (g_bandit.getJointType(i) == bandit::SMART_SERVO)
        g_bandit.setJointPIDConfig(i, 100, 0, 0, -4000, 4001, 50, 0);
    }

    if (g_bandit.syncAllPIDConfigs(5000000))
      ROS_INFO("All modules confirmed PID configuration.\n");
    else 
      ROS_ERROR("Some modules failed to configure\n");

    ros::Rate r(20);

    while (nh.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

  } catch (bandit::BanditException& e)
  {
    ROS_FATAL("Caught bandit exception: %s\n", e.what());
  }

  return 0;
}
