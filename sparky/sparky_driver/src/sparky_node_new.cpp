// preprocessor directives
#include <ros/ros.h>
#include <joint_msgs/JointArray.h>
#include <joint_msgs/Params.h>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <sparky/angle_servo_controller.h>
//#include <sparky/controller.hh>

// angle (degrees and radians) definitions
#define DTOR(rad) ((rad) * M_PI / 180.0)
#define RTOD(deg) ((deg) * 180.0 / M_PI)

// Sparky joint definitions
#define N_JOINTS (19)
#define N_DEVICES (1)
#define N_CHANNELS_EACH (24)
//#define PATH ("/dev/ttyACM0")
std::string g_path = "/dev/ttyACM0";

// global variables
//controller::Controller       g_sparky;
pololu::AngleServoController g_sparky(N_DEVICES, N_CHANNELS_EACH, g_path, true);
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
  ros::NodeHandle nh("~");

  // retrieve ports from parameter server
  //if (!nh.getParam("path", g_path)) nh.setParam("path", g_path);
  /*if (!g_sparky.connect())
  {
    ROS_ERROR("Unable to connect to Sparky at '%s'", g_path.c_str());
    return 1;
  }*/
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
  g_params_res.name[18] = "Base Turn";

  // set remaining parameters
  for (int i = 0; i < n_channels; ++i)
  {
    g_params_res.id[i] = i;
    if (i == 18) // base_turn
    {
      g_params_res.min[i] = DTOR(-335.0);
      g_params_res.max[i] = DTOR(340.0);
    }
    else
    {
      g_params_res.min[i] = 0.0;
      g_params_res.max[i] = DTOR(150.0);
    }
    g_params_res.curr[i] = g_params_res.min[i] + 0.5 * (g_params_res.max[i] - g_params_res.min[i]);//g_params_res.min[i];
  }
  /*
   g_params_res.min[0] = -42.0;
   g_params_res.max[0] = 0.0;
   g_params_res.min[1] = -1.0;
   g_params_res.max[1] = 60.0;
   g_params_res.min[2] = -25.0;
   g_params_res.max[2] = 45.0;
   g_params_res.min[3] = 2.0;
   g_params_res.max[3] = 91.0;
   g_params_res.min[4] = 3.0;
   g_params_res.max[4] = 72.0;
   g_params_res.min[5] = 13.0;
   g_params_res.max[5] = 104.0;
   g_params_res.min[6] = 13.0;
   g_params_res.max[6] = 90.0;
   g_params_res.min[7] = 2.0;
   g_params_res.max[7] = 72.0;
   g_params_res.min[8] = 15.0;
   g_params_res.max[8] = 105.0;
   g_params_res.min[9] = -45.0;
   g_params_res.max[9] = 90.0;
   g_params_res.min[10] = -42.0;
   g_params_res.max[10] = 90.0;
   g_params_res.min[11] = -76.0;
   g_params_res.max[11] = -1.0;
   g_params_res.min[12] = 0.0;
   g_params_res.max[12] = 102.0;
   g_params_res.min[13] = -30.0;
   g_params_res.max[13] = 40.0;
   g_params_res.min[14] = 0.00;
   g_params_res.max[14] = 5.84;
   g_params_res.min[15] = -9.61;
   g_params_res.max[15] = 7.8;
   g_params_res.min[16] = 0.00;
   g_params_res.max[16] = 5.92;
   g_params_res.min[17] = -9.44;
   g_params_res.max[17] = 5.74;
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
//      double vals     - vector of 18 positions
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

bool jointMoveTo(int id, double angle)
{
  //if ((id < 0) || (id >= controller::NChannels))
  if ((id < 0) || (id >= N_JOINTS))
  {
    ROS_ERROR("Joint ID %d out of range!", id);
    return false;
  }

  int device = getJointDevice(id);
  int channel = getJointChannel(id);

  //int min_limit = g_sparky.getAngleMinLimit(device, channel);
  //int max_limit = g_sparky.getAngleMaxLimit(device, channel);
  //int target = min_limit + angle * (max_limit - min_limit); // assumes param [0, 1]
  double target = angle;

  ROS_INFO("Moving joint[%d] (servo[%d, %d]) to %.2f (%.2f)", id, device, channel, RTOD(angle), target);
  if (!g_sparky.setAngleTarget(device, channel, target))
  {
    ROS_ERROR("Error moving joint[%d] (servo[%d, %d] to %.2f (%.2f)!", id, device, channel, RTOD(angle), target);
    return false;
  }

  return true;
} // jointMoveTo(int, double)

int getJointDevice(int id)
{
  return 0;
  return (id < (N_JOINTS / 2)) ? 0 : 1;
} // getJointDevice(int)

int getJointChannel(int id)
{
  return id;
  return 2 * ((getJointDevice(id) == 0) ? id : (id - (N_JOINTS / 2)));
} // getJointChannel(int)

bool initSparky()
{
  int device = -1;
  int channel = -1;
  pololu::MaestroServoController::ServoLimits servo_limits(900, 2100);
  pololu::AngleServoController::AngleServoPair min_limit(0.0, 1100);
  pololu::AngleServoController::AngleServoPair max_limit(DTOR(150.0), 2100);
  pololu::AngleServoController::AngleLimits angle_limits(min_limit, max_limit);
  pololu::MaestroServoController::ServoLimits base_servo_limits(754, 2254);
  pololu::AngleServoController::AngleServoPair base_min_limit(DTOR(-335.0), 754);
  pololu::AngleServoController::AngleServoPair base_max_limit(DTOR(340.0), 2254);
  pololu::AngleServoController::AngleLimits base_angle_limits(base_min_limit, base_max_limit);
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

    if (i == 18) // base_turn
    {
      joint_success = g_sparky.setServoLimits(device, channel, base_servo_limits) && joint_success;
      joint_success = g_sparky.setAngleLimits(device, channel, base_angle_limits) && joint_success;
    }
    else
    {
      joint_success = g_sparky.setServoLimits(device, channel, servo_limits) && joint_success;
      joint_success = g_sparky.setAngleLimits(device, channel, angle_limits) && joint_success;
    }
    joint_success = g_sparky.setServoEnabled(device, channel, enabled) && joint_success;
    joint_success = g_sparky.setServoSpeed(device, channel, speed) && joint_success;
    joint_success = g_sparky.setServoAcceleration(device, channel, accel) && joint_success;
    //joint_success = g_sparky.setServoTarget(device, channel, target) && joint_success;

    if (!joint_success)
      ROS_ERROR("Unable to initialize joint[%d] (servo[%d, %d])...", i, device, channel);
    else
      ROS_INFO("Initialized joint[%d] (servo[%d, %d])!!!", i, device, channel);

    success &= joint_success;
  }

  for (int i = 0, n = N_JOINTS; i < n; ++i)
    success = jointMoveTo(i, g_params_res.curr[i]) && success;
  /*
   //success = true;
   success = jointMoveTo(0, DTOR(110.0)) && success;
   success = jointMoveTo(1, DTOR(35.0)) && success;
   success = jointMoveTo(2, DTOR(80.0)) && success;
   success = jointMoveTo(3, DTOR(30.0)) && success;
   success = jointMoveTo(4, DTOR(75.0)) && success;
   success = jointMoveTo(5, DTOR(60.0)) && success;
   success = jointMoveTo(6, DTOR(30.0)) && success;
   success = jointMoveTo(7, DTOR(85.0)) && success;
   success = jointMoveTo(8, DTOR(70.0)) && success;
   success = jointMoveTo(9, DTOR(120.0)) && success;
   success = jointMoveTo(10, DTOR(120.0)) && success;
   success = jointMoveTo(11, DTOR(35.0)) && success;
   success = jointMoveTo(12, DTOR(90.0)) && success;
   success = jointMoveTo(13, DTOR(80.0)) && success;
   success = jointMoveTo(14, DTOR(90.0)) && success;
   success = jointMoveTo(15, DTOR(115.0)) && success;
   success = jointMoveTo(16, DTOR(0.0)) && success;
   success = jointMoveTo(17, DTOR(0.0)) && success;
   success = jointMoveTo(18, DTOR(0.0)) && success;
   */
  return success;
} // initSparky()
