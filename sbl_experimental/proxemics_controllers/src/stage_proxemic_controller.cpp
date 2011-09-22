// standard includes
#include <iostream>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btMatrix3x3.h>

// ROS includes
#include <angles/angles.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <proxemics_controllers/ProxemicControllerConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// namespaces
using namespace std;

// red (robot) global variables
double g_robot_x = 0.0;
double g_robot_y = 0.0;
double g_robot_th = 0.0;

// purple (parent) global variables
double g_purple_x = 0.0;
double g_purple_y = 0.0;
double g_purple_th = 0.0;

// blue (child) global variables
double g_blue_x = 0.0;
double g_blue_y = 0.0;
double g_blue_th = 0.0;

// laser scan global variables
vector<float> g_ranges;
double g_angle_min = 0.0;
double g_angle_max = 0.0;
double g_angle_inc = 0.0;
double g_min_dist = HUGE_VAL;

// proxemic global variables
double g_social_distance = 1.0;
double g_use_orientation = false;
double g_gain_lin_x = 1.0;
double g_gain_lin_y = 1.0;
double g_gain_ang_z = 1.0;
double g_min_speed_lin_x = 0.01;
double g_min_speed_lin_y = 0.01;
double g_min_speed_ang_z = 0.02;
double g_max_speed_lin_x = 1.0;
double g_max_speed_lin_y = 1.0;
double g_max_speed_ang_z = 0.5 * M_PI;

// function prototypes
double getDistance(double target_x, double target_y, double origin_x = 0.0, double origin_y = 0.0);
double getAngle(double target_x, double target_y, double origin_x = 0.0, double origin_y = 0.0, double origin_th = 0.0);
double sign(double x);

// callback function prototypes
void cbReconfigure(proxemics_controllers::ProxemicControllerConfig &config, uint32_t level);
void cbCloud(const sensor_msgs::PointCloud2ConstPtr &cloud);
void cbLaser(const sensor_msgs::LaserScanConstPtr &scan);
void cbRobot(const nav_msgs::OdometryConstPtr &odom);
void cbPurple(const nav_msgs::OdometryConstPtr &odom);
void cbBlue(const nav_msgs::OdometryConstPtr &odom);

// executes main program code
int main(int argc, char** argv)
{

  // initialize ROS
  ros::init(argc, argv, "stage_proxemic_controller");
  ros::NodeHandle nh;

  // initialize parameters
  nh.setParam("social_distance", g_social_distance);
  nh.setParam("use_orientation", g_use_orientation);
  nh.setParam("g_gain_lin_x", g_gain_lin_x);
  nh.setParam("g_gain_lin_y", g_gain_lin_y);
  nh.setParam("g_gain_ang_z", g_gain_ang_z);
  nh.setParam("g_min_speed_lin_x", g_min_speed_lin_x);
  nh.setParam("g_min_speed_lin_y", g_min_speed_lin_y);
  nh.setParam("g_min_speed_ang_z", g_min_speed_ang_z);
  nh.setParam("g_max_speed_lin_x", g_max_speed_lin_x);
  nh.setParam("g_max_speed_lin_y", g_max_speed_lin_y);
  nh.setParam("g_max_speed_ang_z", g_max_speed_ang_z);

  // initialize dynamic reconfigure parameter server
  dynamic_reconfigure::Server<proxemics_controllers::ProxemicControllerConfig> srv_reconfig;
  dynamic_reconfigure::Server<proxemics_controllers::ProxemicControllerConfig>::CallbackType cb_reconfig;
  cb_reconfig = boost::bind(&cbReconfigure, _1, _2);
  srv_reconfig.setCallback(cb_reconfig);

  // initialize subscribers
  ros::Subscriber sub_cloud = nh.subscribe("input", 1, cbCloud);
  ros::Subscriber sub_laser = nh.subscribe("/robot_0/base_scan", 1, cbLaser);
  ros::Subscriber sub_robot = nh.subscribe("/robot_0/base_pose_ground_truth", 1, cbRobot);
  ros::Subscriber sub_blue = nh.subscribe("/robot_1/base_pose_ground_truth", 1, cbBlue);
  ros::Subscriber sub_purple = nh.subscribe("/robot_2/base_pose_ground_truth", 1, cbPurple);

  // initialize publishers
  ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1);
  //ros::Publisher pub_cmd_vel = nh.advertise < geometry_msgs::Twist > ("/base_controller/command", 1);
  geometry_msgs::Twist cmd_vel;

  //
  tf::TransformListener tl;
  ros::Rate loop_rate(10);

  //
  double range_rp = 0.0;
  double range_rb = 0.0;
  double angle_rp = 0.0;
  double angle_rb = 0.0;
  double angle_pr = 0.0;

  // robot velocities
  double vel_x = 0.0;
  double vel_y = 0.0;
  double vel_th = 0.0;

  while (ros::ok())
  {
    try // get target frame
    {
      if (tl.canTransform("/openni_depth_frame", "/torso", ros::Time(0)))
      {
        tf::StampedTransform rp;
        tl.lookupTransform("/openni_depth_frame", "/torso", ros::Time(0), rp);
        g_purple_x = -rp.getOrigin().z();
        g_purple_y = -rp.getOrigin().x();
        ROS_INFO("purple_x: %0.2f purple_y: %0.2f", g_purple_x, g_purple_y);
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("Transformation failed: [%s]\n", ex.what());
    }

    // get range/angle to purple/blue robots
    range_rp = getDistance(g_purple_x, g_purple_y, g_robot_x, g_robot_y);
    range_rb = getDistance(g_blue_x, g_blue_y, g_robot_x, g_robot_y);
    angle_rp = getAngle(g_purple_x, g_purple_y, g_robot_x, g_robot_y, g_robot_th);
    angle_rb = getAngle(g_blue_x, g_blue_y, g_robot_x, g_robot_y, g_robot_th);
    angle_pr = getAngle(g_robot_x, g_robot_y, g_purple_x, g_purple_y, g_purple_th);

    // Manuela Veloso's (CMU) motion controller
    //vt = pow(sin(angles::from_degrees(angle_rp)), 2) * sign(sin(angles::from_degrees(angle_rp)));
    //vx = pow(cos(angles::from_degrees(angle_rp)), 2) * sign(cos(angles::from_degrees(angle_rp)));

    // proportional control: velocity(error) = gain * error
    //vel_x = g_gain_lin_x * (range_rp - social_distance);
    vel_x = g_gain_lin_x * cos(angle_rp) * (range_rp - g_social_distance);
    if (g_use_orientation)
      vel_y = g_gain_lin_y * sin(sign(angle_pr) * min(abs(angle_pr), 0.5 * M_PI));
    else
      vel_y = 0.0;
    vel_th = g_gain_ang_z * angle_rp;

    // clip linear x velocity
    if (abs(vel_x) < g_min_speed_lin_x)
      vel_x = 0.0;
    else if (abs(vel_x) > g_max_speed_lin_x)
      vel_x = sign(vel_x) * g_max_speed_lin_x;

    // clip linear y velocity
    if (abs(vel_y) < g_min_speed_lin_y)
      vel_y = 0.0;
    else if (abs(vel_y) > g_max_speed_lin_y)
      vel_y = sign(vel_y) * g_max_speed_lin_y;

    // clip angular z velocity
    if (abs(vel_th) < g_min_speed_ang_z)
      vel_th = 0.0;
    else if (abs(vel_th) > g_max_speed_ang_z)
      vel_th = sign(vel_th) * g_max_speed_ang_z;

    // print linear (vel_x and vel_y) and angular (vel_th) velocities
    ROS_INFO("vel = [%.2f, %.2f, %.2f]", vel_x, vel_y, vel_th);

    // publish linear (vel_x and vel_y) and angular (vel_th) velocities
    cmd_vel.linear.x = vel_x;
    cmd_vel.linear.y = vel_y;
    cmd_vel.angular.z = vel_th;
    pub_cmd_vel.publish(cmd_vel);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
} // main(int, char**)

double getDistance(double target_x, double target_y, double origin_x, double origin_y)
{
  return sqrt(pow(target_x - origin_x, 2) + pow(target_y - origin_y, 2));
} // getDistance(double, double, double, double)

double getAngle(double target_x, double target_y, double origin_x, double origin_y, double origin_th)
{
  return angles::normalize_angle(atan2(target_y - origin_y, target_x - origin_x) - origin_th);
} // getAngle(double, double, double, double, double)

double sign(double x)
{
  return ((x < 0.0) ? -1.0 : 1.0);} // sign(double x)

      void
cbReconfigure (proxemics_controllers::ProxemicControllerConfig &config, uint32_t level)
{
  g_social_distance = config.social_distance;
  g_use_orientation = config.use_orientation;
  g_gain_lin_x = config.gain_lin_x;
  g_gain_lin_y = config.gain_lin_y;
  g_gain_ang_z = config.gain_ang_z;
  g_min_speed_lin_x = config.min_speed_lin_x;
  g_min_speed_lin_y = config.min_speed_lin_y;
  g_min_speed_ang_z = config.min_speed_ang_z;
  g_max_speed_lin_x = config.max_speed_lin_x;
  g_max_speed_lin_y = config.max_speed_lin_y;
  g_max_speed_ang_z = config.max_speed_ang_z;
} // cbReconfigure(proxemics_controllers::ProxemicControllerConfig &, uint32_t)

void cbCloud(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud, outputcloud;
  pcl::fromROSMsg(*input, cloud);

  pcl::PointCloud<pcl::PointXYZ>::iterator itr = cloud.begin();
  double dist = HUGE_VAL;
  g_min_dist = dist;
  while (itr != cloud.end())
  {
    double x = itr->x;
    //double y = itr->y;
    double z = itr->z;
    dist = getDistance(x, z);

    if (dist < g_min_dist)
      g_min_dist = dist;
    ++itr;
  }

  cout << g_min_dist << endl;
} // cbCloud(const sensor_msgs::PointCloud2ConstPtr &)

void cbLaser(const sensor_msgs::LaserScanConstPtr &scan)
{
  g_angle_min = scan->angle_min;
  g_angle_max = scan->angle_max;
  g_angle_inc = scan->angle_increment;
  g_ranges = scan->ranges;
} // cbLaser(const sensor_msgs::LaserScanConstPtr &)

void cbRobot(const nav_msgs::OdometryConstPtr &odom)
{
  g_robot_y = odom->pose.pose.position.x;
  g_robot_x = -odom->pose.pose.position.y;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  btQuaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,
                 odom->pose.pose.orientation.w);
  btMatrix3x3(q).getRPY(roll, pitch, yaw);
  g_robot_th = angles::normalize_angle(yaw + M_PI / 2.0);
} // cbRobot(const nav_msgs::OdometryConstPtr &)

void cbPurple(const nav_msgs::OdometryConstPtr &odom)
{
  g_purple_y = odom->pose.pose.position.x;
  g_purple_x = -odom->pose.pose.position.y;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  btQuaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,
                 odom->pose.pose.orientation.w);
  btMatrix3x3(q).getRPY(roll, pitch, yaw);
  g_purple_th = angles::normalize_angle(yaw + M_PI / 2.0);
} // cbPurple(const nav_msgs::OdometryConstPtr &)

void cbBlue(const nav_msgs::OdometryConstPtr &odom)
{
  g_blue_y = odom->pose.pose.position.x;
  g_blue_x = -odom->pose.pose.position.y;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  btQuaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,
                 odom->pose.pose.orientation.w);
  btMatrix3x3(q).getRPY(roll, pitch, yaw);
  g_blue_th = angles::normalize_angle(yaw + M_PI / 2.0);
} // cbBlue(const nav_msgs::OdometryConstPtr &)
