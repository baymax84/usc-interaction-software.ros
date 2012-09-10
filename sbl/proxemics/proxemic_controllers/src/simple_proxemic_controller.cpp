// standard includes
#include <iostream>

// ROS includes
#include <angles/angles.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <proxemic_controllers/ProxemicControllerConfig.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

// namespaces
using namespace std;

// proxemic global variables
double g_social_distance = 1.0;
double g_angle_robot_to_person = 0.0;
double g_angle_person_to_robot = 0.0;
std::string g_origin_frame = "/base_link";
std::string g_target_frame = "/proxemics_target";
bool g_use_orientation = false;
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
void cbReconfigure(proxemic_controllers::ProxemicControllerConfig &config, uint32_t level);

// executes main program code
int main(int argc, char** argv)
{

  // initialize ROS
  ros::init(argc, argv, "simple_proxemic_controller");
  ros::NodeHandle nh;

  // initialize parameters
  nh.setParam("social_distance", g_social_distance);
  nh.setParam("angle_robot_to_person", g_angle_robot_to_person);
  nh.setParam("angle_person_to_robot", g_angle_person_to_robot);
  nh.setParam("origin_frame", g_origin_frame);
  nh.setParam("target_frame", g_target_frame);
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
  dynamic_reconfigure::Server<proxemic_controllers::ProxemicControllerConfig> srv_reconfig;
  dynamic_reconfigure::Server<proxemic_controllers::ProxemicControllerConfig>::CallbackType cb_reconfig;
  cb_reconfig = boost::bind(&cbReconfigure, _1, _2);
  srv_reconfig.setCallback(cb_reconfig);

  // initialize publishers
  ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
  geometry_msgs::Twist cmd_vel;

  //
  tf::TransformListener tf_listener;
  ros::Rate loop_rate(30);

  //
  double range_rp = 0.0;
  double angle_rp = 0.0;
  double angle_pr = 0.0;
  double vel_r = 0.0;
  double vel_x = 0.0;
  double vel_y = 0.0;
  double vel_th = 0.0;

  while (ros::ok())
  {
    vel_x = vel_y = vel_th = 0.0;
    ros::Time t_now = ros::Time::now();

    try // get person position
    {
      if (tf_listener.canTransform(g_origin_frame, g_target_frame, ros::Time(0)))
      {
        tf::StampedTransform tf_rp;
        tf::StampedTransform tf_pr;
        tf_listener.waitForTransform(g_origin_frame, g_target_frame, t_now, ros::Duration(0.25));
        tf_listener.lookupTransform(g_origin_frame, g_target_frame, t_now, tf_rp);
        tf_listener.lookupTransform(g_target_frame, g_origin_frame, t_now, tf_pr);

    // +
    // v
    //
    //
    // ^
    // +

        // get range/angle to target frame
        range_rp = getDistance(tf_rp.getOrigin().x(), tf_rp.getOrigin().y());
        angle_rp = getAngle(tf_rp.getOrigin().x(), tf_rp.getOrigin().y());
        angle_pr = getAngle(tf_pr.getOrigin().x(), tf_pr.getOrigin().y());

        // proportional control: velocity(error) = gain * error
        //vel_x = g_gain_lin_x * (range_rp - g_social_distance);

        double a1 = angle_rp - g_angle_robot_to_person;
        double a2 = -angle_rp;

        vel_x = g_gain_lin_x * cos( a1 ) * ( range_rp - g_social_distance );
        if ( g_use_orientation ) vel_y = g_gain_lin_y * sin( sign( angle_pr - g_angle_person_to_robot ) * min( abs( angle_pr - g_angle_person_to_robot ), M_PI_2 ) );
        //vel_th = g_gain_ang_z * angle_rp;

        // proportional control: velocity(error) = gain * error
        //vel_x = g_gain_lin_x * cos(angle_rp) * (range_rp - g_social_distance);
        //if (g_use_orientation)
        //  vel_y = g_gain_lin_y * sin(sign(angle_pr - g_angle_person_to_robot) * min(abs(angle_pr - g_angle_person_to_robot), 0.5 * M_PI));
        //vel_th = g_gain_ang_z * (angle_rp - g_angle_robot_to_person);

        //ROS_INFO( "%.2f %.2f", g_angle_robot_to_person, angle_rp );

        //ROS_INFO( "%.2f", a1 );
        //ROS_INFO( "%.2f", a2 );

        vel_th = g_gain_ang_z * a1;

        // "rotate" velocities based on desired angle from robot to person
        double vel_x_p = vel_x * cos( a2 ) + vel_y * sin( a2 );
        double vel_y_p = -vel_x * sin( a2 ) + vel_y * cos( a2 );
        //ROS_INFO("vel = [%.2f, %.2f, %.2f]", vel_x, vel_y, vel_th);
        vel_x = vel_x_p;
        vel_y = vel_y_p;
        //ROS_INFO("vel = [%.2f, %.2f, %.2f]", vel_x, vel_y, vel_th);

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
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }

    // print linear (vel_x and vel_y) and angular (vel_th) velocities
    //ROS_INFO("vel = [%.2f, %.2f, %.2f]", vel_x, vel_y, vel_th);
    //ROS_INFO( "----------" );

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
cbReconfigure (proxemic_controllers::ProxemicControllerConfig &config, uint32_t level)
{
  g_social_distance = config.social_distance;
  g_angle_robot_to_person = config.angle_robot_to_person;
  g_angle_person_to_robot = config.angle_person_to_robot;
  g_origin_frame = config.origin_frame;
  g_target_frame = config.target_frame;
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
} // cbReconfigure(proxemic_controllers::ProxemicControllerConfig &, uint32_t)
