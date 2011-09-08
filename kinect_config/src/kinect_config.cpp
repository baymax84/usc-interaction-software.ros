// standard includes
#include <iostream>

// ROS includes
#include <dynamic_reconfigure/server.h>
#include <kinect_config/KinectAuxConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>

// namespaces
using namespace std;

// global variables (parameters)
std_msgs::Float64 g_tilt_angle;
bool g_maintain_tilt_angle = false;
std_msgs::UInt16 g_led_option;
double g_loop_rate = 0.0;
bool g_show_imu = false;
bool g_show_cur_tilt_angle = false;
bool g_show_cur_tilt_status = false;
sensor_msgs::Imu g_imu;
double g_cur_tilt_angle = 0.0;
int g_cur_tilt_status = 0;
bool g_update = false;

// callback function prototypes
void cbImu(const sensor_msgs::ImuConstPtr &imu);
void cbCurTiltAngle(const std_msgs::Float64ConstPtr &angle);
void cbCurTiltStatus(const std_msgs::UInt8ConstPtr &status);
void cbReconfigure(kinect_config::KinectAuxConfig &config, uint32_t level);

// executes main program code
int main(int argc, char** argv)
{

  // initialize ROS
  ros::init(argc, argv, "kinect_config");
  ros::NodeHandle nh;

  // initialize parameters
  nh.setParam("tilt_angle", g_tilt_angle.data);
  nh.setParam("maintain_tilt_angle", g_maintain_tilt_angle);
  nh.setParam("led_option", g_led_option.data);
  nh.setParam("loop_rate", g_loop_rate);
  nh.setParam("show_imu", g_show_imu);
  nh.setParam("show_cur_tilt_angle", g_show_cur_tilt_angle);
  nh.setParam("show_cur_tilt_status", g_show_cur_tilt_status);
  g_update = true;

  // initialize dynamic reconfigure parameter server
  dynamic_reconfigure::Server<kinect_config::KinectAuxConfig> srv_reconfig;
  dynamic_reconfigure::Server<kinect_config::KinectAuxConfig>::CallbackType cb_reconfig;
  cb_reconfig = boost::bind(&cbReconfigure, _1, _2);
  srv_reconfig.setCallback(cb_reconfig);

  // initialize publishers
  ros::Publisher pub_tilt_angle = nh.advertise<std_msgs::Float64>("/tilt_angle", 1);
  ros::Publisher pub_led_option = nh.advertise<std_msgs::UInt16>("/led_option", 1);

  // initialize subscribers
  ros::Subscriber sub_imu = nh.subscribe("/imu", 1, cbImu);
  ros::Subscriber sub_cur_tilt_angle = nh.subscribe("/cur_tilt_angle", 1, cbCurTiltAngle);
  ros::Subscriber sub_cur_tilt_status = nh.subscribe("/cur_tilt_status", 1, cbCurTiltStatus);

  while (ros::ok())
  {
    if (g_update)
    {
      pub_tilt_angle.publish(g_tilt_angle);
      pub_led_option.publish(g_led_option);
      g_update = false;
    }
    else if (g_maintain_tilt_angle)
      pub_tilt_angle.publish(g_tilt_angle);
    if (g_show_imu) ROS_WARN("Showing IMU data is not yet implemented...");
    if (g_show_cur_tilt_angle) ROS_INFO("cur_tilt_angle = %.2f", g_cur_tilt_angle);
    if (g_show_cur_tilt_status) ROS_INFO("cur_tilt_status = %d", g_cur_tilt_status);
    ros::spinOnce();
    ros::Rate(g_loop_rate).sleep();
  }

  return 0;
} // main(int, char**)

void cbImu(const sensor_msgs::ImuConstPtr &imu)
{
  // ...
} // cbImu(const sensor_msgs::ImuConstPtr &)

void cbCurTiltAngle(const std_msgs::Float64ConstPtr &angle)
{
  g_cur_tilt_angle = angle->data;
} // cbCurTiltAngle(const std_msgs::Float64ConstPtr &)

void cbCurTiltStatus(const std_msgs::UInt8ConstPtr &status)
{
  g_cur_tilt_status = status->data;
} // cbCurTiltStatus(const std_msgs::UInt8ConstPtr &)

void cbReconfigure(kinect_config::KinectAuxConfig &config, uint32_t level)
{
  g_tilt_angle.data = config.tilt_angle;
  g_maintain_tilt_angle = config.maintain_tilt_angle;
  g_led_option.data = config.led_option;
  g_loop_rate = config.loop_rate;
  g_show_imu = config.show_imu;
  g_show_cur_tilt_angle = config.show_cur_tilt_angle;
  g_show_cur_tilt_status = config.show_cur_tilt_status;
  g_update = true;
} // cbReconfigure(kinect_config::KinectAuxConfig &, uint32_t)
