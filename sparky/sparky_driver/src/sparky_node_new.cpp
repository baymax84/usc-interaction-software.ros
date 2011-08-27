// preprocessor directives
#include <ros/ros.h>
#include <joint_msgs/JointArray.h>
#include <joint_msgs/Params.h>
#include <sensor_msgs/JointState.h>
//#include <sparky/controller.hh>
#include <sparky/maestro_servo_controller.h>
#include <math.h>

// angle (degrees and radians) definitions
#define DTOR(rad) ((rad) * M_PI / 180.0f)
#define RTOD(deg) ((deg) * 180.0f / M_PI)

// Sparky joint definitions
#define N_JOINTS (18)
#define N_DEVICES (2)
#define N_CHANNELS_EACH (24)
#define PATH ("/dev/ttyACM0")

// global variables
//controller::Controller       g_sparky;
pololu::MaestroServoController g_sparky(N_DEVICES, N_CHANNELS_EACH, PATH);
joint_msgs::Params::Response g_params_res;

// function prototypes
bool initParams();
bool initServoController();
void shutdownServoController();
bool outputPositions(double* vals, bool* active);
bool jointMoveTo(int id, double angle);
int getJointDevice(int id);
int getJointChannel(int id);
bool initSparky();

//
bool paramsCallback(joint_msgs::Params::Request &req, joint_msgs::Params::Response &res)
{
  res = g_params_res;
  return true;
} // paramsCallback(joint_msgs::Params::{Request, Response} &)

//
void jointCallback(const joint_msgs::JointConstPtr &joint)
{
  if (joint->active)
  {
    //ROS_INFO("Moving joint[%d] to %.2f", joint->id, joint->angle);
    if (!jointMoveTo(joint->id, joint->angle))
      ROS_ERROR("Error moving joint[%d]!", joint->id);
  }
} // jointCallback(const joint_msgs::JointConstPtr &)

//
void jointArrayCallback(const joint_msgs::JointArrayConstPtr &joints)
{
  for (int i = 0, n = joints->joints.size(); i < n; ++i)
  {
    if (joints->joints[i].active)
    {
      //ROS_INFO("Moving joint[%d] to %.2f", joints->joints[i].id, joints->joints[i].angle);
      if (!jointMoveTo(joints->joints[i].id, joints->joints[i].angle))
        ROS_ERROR("Error moving joint[%d]!", joints->joints[i].id);
    }
  }
} // jointArrayCallback(const joint_msgs::JointArrayConstPtr &)

//
void stateCallback(ros::Publisher &joint_pub)
{
} // stateCallback(ros::Publisher &)

//
int main(int argc, char** argv)
{

  // initialize ROS node
  ros::init(argc, argv, "sparky");
  ros::NodeHandle nh;

  // retrieve ports from parameter server
  //std::string ports[0];
  //nh.param("port0", ports[0], std::string("/dev/ttyUSB0"));
  //nh.param("port1", ports[1], std::string("/dev/ttyUSB1"));

  // set up ROS publishers
  //ros::Publisher joints_pub = nh.advertise<sensor_msgs::JointState>(
  //                              "joint_states", 1000);
  //ros::Publisher joint_pub = nh.advertise<joint_msgs::Joint>("joint_cmd", 1000);
  //ros::Publisher joints_pub = nh.advertise<joint_msgs::JointArray>("joints_cmd", 1000);

  // set up ROS subscribers
  //ros::Subscriber joints_sub = nh.subscribe("joints_cmd", 1, jointArrayCallback);
  ros::Subscriber joint_sub = nh.subscribe("joint_cmd", 1000, jointCallback);

  // set up ROS services
  initParams();
  ros::ServiceServer params_srv = nh.advertiseService("sparky_params", paramsCallback);

  //
  if (!initServoController())
    return 1;

  initSparky();

  ros::spin();

  // exit program
  shutdownServoController();
  return 0;
} // main(int, char**)

//
bool initParams()
{

  // resize the parameter vectors
  const int n_channels = N_JOINTS; //controller::NChannels;
  g_params_res.name.resize(n_channels);
  g_params_res.id.resize(n_channels);
  g_params_res.min.resize(n_channels);
  g_params_res.max.resize(n_channels);
  g_params_res.curr.resize(n_channels);

  // set parameter names
  g_params_res.name[0] = "Mouth";
  g_params_res.name[1] = "Head Nod";
  g_params_res.name[2] = "Head Turn";
  g_params_res.name[3] = "Right Arm Forw";
  g_params_res.name[4] = "Right Arm Out";
  g_params_res.name[5] = "Right Elbow";
  g_params_res.name[6] = "Left Arm Forw";
  g_params_res.name[7] = "Left Arm Out";
  g_params_res.name[8] = "Left Elbow";
  g_params_res.name[9] = "Right Wrist";
  g_params_res.name[10] = "Left Wrist";
  g_params_res.name[11] = "Body Forw";
  g_params_res.name[12] = "Eyelids";
  g_params_res.name[13] = "Eye Left/Right";
  g_params_res.name[14] = "Right Foot Up";
  g_params_res.name[15] = "Right Foot Forward";
  g_params_res.name[16] = "Left Foot Up";
  g_params_res.name[17] = "Left Foot Forward";

  // set remaining parameters
  for (int i = 0; i < n_channels; ++i)
  {
    g_params_res.id[i] = i;
    g_params_res.min[i] = 0.0f;
    g_params_res.max[i] = 1.0f;
    g_params_res.curr[i] = 0.5f;//g_params_res.min[i]; //0.005f;
  }
  /*
   g_params_res.min[0] = -42.0f;
   g_params_res.max[0] = 0.0f;
   g_params_res.min[1] = -1.0f;
   g_params_res.max[1] = 60.0f;
   g_params_res.min[2] = -25.0f;
   g_params_res.max[2] = 45.0f;
   g_params_res.min[3] = 2.0f;
   g_params_res.max[3] = 91.0f;
   g_params_res.min[4] = 3.0f;
   g_params_res.max[4] = 72.0f;
   g_params_res.min[5] = 13.0f;
   g_params_res.max[5] = 104.0f;
   g_params_res.min[6] = 13.0f;
   g_params_res.max[6] = 90.0f;
   g_params_res.min[7] = 2.0f;
   g_params_res.max[7] = 72.0f;
   g_params_res.min[8] = 15.0f;
   g_params_res.max[8] = 105.0f;
   g_params_res.min[9] = -45.0f;
   g_params_res.max[9] = 90.0f;
   g_params_res.min[10] = -42.0f;
   g_params_res.max[10] = 90.0f;
   g_params_res.min[11] = -76.0f;
   g_params_res.max[11] = -1.0f;
   g_params_res.min[12] = 0.0f;
   g_params_res.max[12] = 102.0f;
   g_params_res.min[13] = -30.0f;
   g_params_res.max[13] = 40.0f;
   g_params_res.min[14] = 0.00f;
   g_params_res.max[14] = 5.84f;
   g_params_res.min[15] = -9.61f;
   g_params_res.max[15] = 7.8f;
   g_params_res.min[16] = 0.00f;
   g_params_res.max[16] = 5.92f;
   g_params_res.min[17] = -9.44f;
   g_params_res.max[17] = 5.74f;
   */
  return true;
} // initParams()

//////////////////////////////////////////////////////////////////////
//
//  initServoController()
//
//  connects to and initializes the Pololu controllers for the minimatronic
//  returns true on success, false on failure
//
//  no arguements
//

bool initServoController()
{
  ROS_INFO("Initializing servo controller...\n");
  g_sparky.connect();
  if (!g_sparky.isConnected())
  {
    ROS_ERROR("Unable to open controller, exiting \n");
    return false;
  }

  ROS_INFO("Servo controller initialized successfully! \n");

  // success
  return true;
} // initServoController();

//////////////////////////////////////////////////////////////////////
//
//  shutdownServoController()
//
//  shuts down and disconnects the Pololu controllers for the minimatronic
//  returns no value (void)
//
//  no arguements
//

void shutdownServoController()
{
  ROS_INFO("Shutting down servo controller... \n");

  // move to home (mid stroke)
  for (int i = 0, n = g_sparky.getNumDevices(); i < n; ++i)
    g_sparky.setServosHome(i);

  // allow some time to get there before we shutdown
  sleep(1);
  //g_sparky.waitForServosDone();

  // turn off the controllers
  //g_sparky.turnOffAll();

  // close (the close would be done automatically anyway by the dtor)
  g_sparky.disconnect();
  ROS_INFO("Servo controller shut down successfully!\n");
} // shutdownServoController

//////////////////////////////////////////////////////////////////////
//
//  outputPositions(double* vals, bool* active)
//
//  commands the positions of the mimiatronic's 18 motors
//  returns true on success, false on failure
//
//  arguements:
//      douible vals     - vertor of 18 positions
//      bool active      - vector of 18 bools (if true,  use the position val
//                                             if false, ignore the position val (hold current pos))
//

bool outputPositions(double* vals, bool* active)
{
  for (int i = 0, n = N_JOINTS; i < n; ++i)
  {
    if (active == 0 || active[i])
    {
      if (!jointMoveTo(i, vals[i]))
      {
        ROS_ERROR("Error moving %d (servo[%d, %d])", i, getJointDevice(i), getJointChannel(i));
        return false;
      }
    }
  }

  // success
  return true;
} // outputPositions(double*, bool*)

bool jointMoveTo(int id, double param) //double angle)
{
  //if ((id < 0) || (id >= controller::NChannels))
  if ((id < 0) || (id >= N_JOINTS))
  {
    ROS_ERROR("Joint ID %d out of range!", id);
    return false;
  }

  int device = getJointDevice(id);
  int channel = getJointChannel(id);

  /*
   double position = 0, z = 0;
   if (id == 0)
   {
   ROS_INFO("Mouth");
   z = (angle + 18) / 15;
   position = 0.0030 * pow(z, 6) + 0.0040 * pow(z, 5) + 0.0054 * pow(z, 4) - 0.0098 * pow(z, 3) - 0.0580 * pow(z, 2)
   - 0.1260 * pow(z, 1) + 0.4206;
   //position=(angle-46.7488628411476f)/113.6196641077f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 1)
   {
   ROS_INFO("Head Nod");
   z = (angle - 29) / 22;
   position = -0.0666 * pow(z, 6) + 0.0109 * pow(z, 5) + 0.1640 * pow(z, 4) + 6.5731e-04 * pow(z, 3)
   - 0.0721 * pow(z, 2) + 0.1682 * pow(z, 1) + 0.6943;
   //position=(angle-38.3333333333333f)/115.714285714286f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 2)
   {
   ROS_INFO("Head Turn");
   z = (angle - 5.2) / 27;
   position = -0.1167 * pow(z, 6) + 0.1532 * pow(z, 5) + 0.1681 * pow(z, 4) - 0.1587 * pow(z, 3) - 0.0665 * pow(z, 2)
   + 0.2005 * pow(z, 1) + 0.6360;
   //position=(angle+69.6944444444444f)/120.833333333333f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 3)
   {
   ROS_INFO("Right Arm Forw");
   z = (angle - 48) / 35;
   position = 0.0465 * pow(z, 6) + 0.0776 * pow(z, 5) - 0.1033 * pow(z, 4) - 0.0777 * pow(z, 3) + 0.0565 * pow(z, 2)
   + 0.2056 * pow(z, 1) + 0.6247;
   //position=(angle-131.81111111111f)/-141.851851851852f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 4)
   {
   ROS_INFO("Right Arm Out");
   z = (angle - 39) / 25;
   position = 0.0091 * pow(z, 6) + 0.0064 * pow(z, 5) - 0.0215 * pow(z, 4) - 0.0315 * pow(z, 3) - 0.0047 * pow(z, 2)
   - 0.1399 * pow(z, 1) + 0.6374;
   //position=(angle-219.549647417572f)/-144.139508290452f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 5)
   {
   ROS_INFO("Right Elbow");
   z = (angle - 58) / 32;
   position = 0.0314 * pow(z, 6) + 0.0027 * pow(z, 5) - 0.1092 * pow(z, 4) + 0.0036 * pow(z, 3) + 0.1033 * pow(z, 2)
   + 0.1592 * pow(z, 1) + 0.5885;
   //position=(angle-234.478603393623f)/-184.882528435577f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 6)
   {
   ROS_INFO("Left Arm Forward");
   z = (angle - 48) / 27;
   position = 0.0062 * pow(z, 6) - 2.2328e-04 * pow(z, 5) - 0.0171 * pow(z, 4) + 0.0063 * pow(z, 3)
   - 0.0083 * pow(z, 2) + 0.1638 * pow(z, 1) + 0.6417;
   //position=(angle-138.73899869476f)/-154.09752004475f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 7)
   {
   ROS_INFO("Left Arm Out");
   z = (angle - 39) / 24;
   position = -0.03178 * pow(z, 6) - 1.1871e-04 * pow(z, 5) + 0.1067 * pow(z, 4) - 0.0120 * pow(z, 3)
   - 0.0911 * pow(z, 2) - 0.1343 * pow(z, 1) + 0.6027;
   //position=(angle-225.198577824599f)/-163.286805372664f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 8)
   {
   ROS_INFO("Left Elbow");
   z = (angle - 59) / 31;
   position = 0.0251 * pow(z, 6) - 0.0074 * pow(z, 5) - 0.0787 * pow(z, 4) + 0.0243 * pow(z, 3) + 0.0553 * pow(z, 2)
   + 0.1596 * pow(z, 1) + 0.5651;
   //position=(angle-221.268404907975f)/-176.402278702892f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 9)
   {
   ROS_INFO("Right Wrist");
   z = (angle - 28) / 49;
   position = 0.0282 * pow(z, 6) - 0.0092 * pow(z, 5) - 0.0954 * pow(z, 4) + 0.0073 * pow(z, 3) + 0.0617 * pow(z, 2)
   - 0.1738 * pow(z, 1) + 0.6084;
   //position=(angle-275.015397536393f)/-259.658454647255f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 10)
   {
   ROS_INFO("Left Wrist");
   z = (angle - 32) / 50;
   position = 0.0157 * pow(z, 6) - 0.0065 * pow(z, 5) - 0.0743 * pow(z, 4) - 0.0683 * pow(z, 3) + 0.0129 * pow(z, 2)
   - 0.1286 * pow(z, 1) + 0.5936;
   //position=(angle-252.358992154496f)/-230.890162945081f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 11)
   {
   ROS_INFO("Body Forw");
   z = (angle + 38) / 29;
   position = -0.0380 * pow(z, 6) - 0.04068 * pow(z, 5) + 0.1093 * pow(z, 4) + 0.0038 * pow(z, 3) - 0.0979 * pow(z, 2)
   - 0.1619 * pow(z, 1) + 0.6570;
   //position=(angle-41.9785136979112f)/-126.08507400021f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 12)
   {
   ROS_INFO("Eyelids");
   z = (angle - 41) / 37;
   position = 0.0179 * pow(z, 6) - 0.0401 * pow(z, 5) - 0.0193 * pow(z, 4) + 0.1019 * pow(z, 3) - 0.0554 * pow(z, 2)
   + 0.0697 * pow(z, 1) + 0.7830;
   //position= //log((angle/0.174040401645988f))/log(1264.93953011707f)//(angle+169.36599634369f)/292.614259597802f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 13)
   {
   ROS_INFO("Eyes Left/Right");
   z = (angle - 0.22) / 25;
   position = 0.0286 * pow(z, 6) - 0.0611 * pow(z, 5) - 0.0450 * pow(z, 4) + 0.1518 * pow(z, 3) - 0.0300 * pow(z, 2)
   + 0.0558 * pow(z, 1) + 0.5385;
   //position=(angle+92.5977777777777f)/182.0f;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 14)
   {
   ROS_INFO("Right Foot Up");
   z = (angle - 3.1) / 2.2;
   position = -0.0056 * pow(z, 6) - 0.0164 * pow(z, 5) - 0.0012 * pow(z, 4) - 0.0095 * pow(z, 3) - 0.0030 * pow(z, 2)
   - 0.1536 * pow(z, 1) + 0.4547;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 15)
   {
   ROS_INFO("Right Foot Foward");
   z = (angle - 0.44) / 6.8;
   position = -0.1955 * pow(z, 6) + 0.0206 * pow(z, 5) + 0.5468 * pow(z, 4) - 0.1139 * pow(z, 3) - 0.4521 * pow(z, 2)
   - 0.1703 * pow(z, 1) + 0.4749;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 16)
   {
   ROS_INFO("Left Foot Up");
   z = (angle - 3.1) / 2.3;
   position = 0.0202 * pow(z, 6) - 0.0050 * pow(z, 5) - 0.0593 * pow(z, 4) - 0.0458 * pow(z, 3) + 0.0251 * pow(z, 2)
   - 0.1369 * pow(z, 1) + 0.4743;
   ROS_INFO("Servo Position %f", position);
   }
   else if (id == 17)
   {
   ROS_INFO("Left Foot Forward");
   z = (angle + 1.6) / 5.8;
   position = -0.0784 * pow(z, 8) - 0.1103 * pow(z, 7) + 0.2432 * pow(z, 6) + 0.2296 * pow(z, 5) - 0.2460 * pow(z, 4)
   - 0.1307 * pow(z, 3) + 0.0595 * pow(z, 2) - 0.2076 * pow(z, 1) + 0.3943;
   ROS_INFO("Servo Position %f", position);
   }
   else
   position = angle;
   if ((angle < g_params_res.min[id]) || (angle > g_params_res.max[id]))
   {
   ROS_ERROR("Joint angle %.2f out of range!", angle);
   return false;
   }
   */

  int min_limit = g_sparky.getServoMinLimit(device, channel);
  int max_limit = g_sparky.getServoMaxLimit(device, channel);
  int target = min_limit + param * (max_limit - min_limit); // assumes param [0, 1]

  ROS_INFO("Moving joint[%d] (servo[%d, %d]) to %.2f (%d)", id, device, channel, param, target);
  if (!g_sparky.setServoTarget(device, channel, target))
  {
    ROS_ERROR("Error moving joint[%d] (servo[%d, %d] to %.2f (%d)!", id, device, channel, param, target);
    return false;
  }

  return true;
} // jointMoveTo(int, double)

int getJointDevice(int id)
{
  return (id < (N_JOINTS / 2)) ? 0 : 1;
} // getJointDevice(int)

int getJointChannel(int id)
{
  return 2 * ((getJointDevice(id) == 0) ? id : (id - (N_JOINTS / 2)));
} // getJointChannel(int)

bool initSparky()
{
  int device = -1;
  int channel = -1;
  pololu::MaestroServoController::ServoLimits servo_limits(900, 2100);
  bool enabled = true;
  int speed = 0;
  int accel = 0;
  //int target = 0;

  bool success = true;
  for (int i = 0, n = N_JOINTS; i < n; ++i)
  {
    bool joint_success = true;
    device = getJointDevice(i);
    channel = getJointChannel(i);

    int sleep_time = 10000;
    joint_success = g_sparky.setServoLimits(device, channel, servo_limits) && joint_success;
    usleep(sleep_time);
    joint_success = g_sparky.setServoEnabled(device, channel, enabled) && joint_success;
    usleep(sleep_time);
    joint_success = g_sparky.setServoSpeed(device, channel, speed) && joint_success;
    usleep(sleep_time);
    joint_success = g_sparky.setServoAcceleration(device, channel, accel) && joint_success;
    usleep(sleep_time);
    //joint_success = g_sparky.setServoTarget(device, channel, target) && joint_success;

    if (!joint_success)
      ROS_ERROR("Unable to initialize joint[%d] (servo[%d, %d])...", i, device, channel);
    else
      ROS_INFO("Initialized joint[%d] (servo[%d, %d])!!!", i, device, channel);

    success &= joint_success;
  }

  //success = true;
  success = jointMoveTo(0, 0.25f) && success;
  success = jointMoveTo(1, 0.50f) && success;
  success = jointMoveTo(2, 0.50f) && success;
  success = jointMoveTo(3, 0.25f) && success;
  success = jointMoveTo(4, 0.25f) && success;
  success = jointMoveTo(5, 0.25f) && success;
  success = jointMoveTo(6, 0.33f) && success;
  success = jointMoveTo(7, 0.25f) && success;
  success = jointMoveTo(8, 0.25f) && success;
  success = jointMoveTo(9, 0.25f) && success;
  success = jointMoveTo(10, 0.60f) && success;
  success = jointMoveTo(11, 0.58f) && success;
  success = jointMoveTo(12, 0.58f) && success;
  success = jointMoveTo(13, 0.57f) && success;
  success = jointMoveTo(14, 0.00f) && success;
  success = jointMoveTo(15, 0.37f) && success;
  success = jointMoveTo(16, 0.05f) && success; // standing upright
  success = jointMoveTo(17, 0.40f) && success;

  return success;
} // initSparky()
