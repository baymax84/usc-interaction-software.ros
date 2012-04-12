// preprocessor directives
#include <ros/ros.h>
#include <joint_msgs/JointArray.h>
#include <joint_msgs/Params.h>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <sparky/servo_angle_controller.h>
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
pololu::maestro::ServoAngleController g_sparky(N_DEVICES, N_CHANNELS_EACH, g_path, true);
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
  double servo_min = 0.0;
  double servo_max = 0.0;
  double joint_min = 0.0;
  double joint_max = 0.0;
  double joint_home = 0.0;
  for (int i = 0; i < n_channels; ++i)
  {
    g_params_res.id[i] = i;
    switch (i)
    {
		case  0:	// Mouth
			joint_min = 34.0;
			joint_max = 0.0;
			servo_min = 80.0;
			servo_max = 150.0;
			joint_home = joint_min;
			break;
		case  1:	// Head Nod
			joint_min = 96.0;
			joint_max = 40.0;
			servo_min = 51.0;
			servo_max = 150.0;
			joint_home = 90.0;
			break;
		case  2:	// Head Turn
			joint_min = -35.0;
			joint_max = 38.0;
			servo_min = 48.0;
			servo_max = 150.0;
			joint_home = 0.5 * (joint_min + joint_max);
			break;
		case  3:	// Right Arm Forw
			joint_min = 12.0;
			joint_max = 96.0;
			servo_min = 28.0;
			servo_max = 150.0;
			joint_home = joint_min;
			break;
		case  4:	// Right Arm Out
			joint_min = 15.0;
			joint_max = 91.0;
			servo_min = 36.0;
			servo_max = 150.0;
			joint_home = joint_min;
			break;
		case  5:	// Right Elbow
			joint_min = 12.0;
			joint_max = 105.0;
			servo_min = 23.0;
			servo_max = 150.0;
			joint_home = joint_min;
			break;
		case  6:	// Left Arm Forw
			joint_min = 13.0;
			joint_max = 101.0;
			servo_min = 24.0;
			servo_max = 150.0;
			joint_home = joint_min;
			break;
		case  7:	// Left Arm Out
			joint_min = 13.0;
			joint_max = 88.0;
			servo_min = 22.0;
			servo_max = 150.0;
			joint_home = joint_min;
			break;
		case  8:	// Left Elbow
			joint_min = 0.0;
			joint_max = 104.0;
			servo_min = 22.0;
			servo_max = 150.0;
			joint_home = joint_min;
			break;
		case  9:	// Right Wrist
			joint_min = -86.0;
			joint_max = 55.0;
			servo_min = 38.0;
			servo_max = 150.0;
			joint_home = 0.5 * (joint_min + joint_max);
			break;
		case 10:	// Left Wrist
			joint_min = -90.0;
			joint_max = 45.0;
			servo_min = 28.0;
			servo_max = 150.0;
			joint_home = 0.5 * (joint_min + joint_max);
			break;
		case 11:	// Body Forw
			joint_min = 19.0;
			joint_max = 85.0;
			servo_min = 30.0;
			servo_max = 150.0;
			joint_home = joint_min;
			break;
		case 12:	// Eyelids
			joint_min = 130.0;
			joint_max = 0.0;
			servo_min = 62.0;
			servo_max = 150.0;
			joint_home = 0.5 * (joint_min + joint_max);
			break;
		case 13:	// Eye Left/Right
			joint_min = -45.0;//-35.0;
			joint_max = 45.0;//44.0;
			servo_min = 46.0;
			servo_max = 150.0;
			joint_home = 0.5 * (joint_min + joint_max);
			break;
		case 14:	// Right Foot Up
			joint_min = 0.0;
			joint_max = 5.9;
			servo_min = 10.0;
			servo_max = 122.0;
			joint_home = joint_min;
			break;
		case 15:	// Right Foot Forward
			joint_min = 35.0;
			joint_max = -52.0;
			servo_min = 18.0;
			servo_max = 150.0;
			joint_home = 0.5 * (joint_min + joint_max);
			break;
		case 16:	// Left Foot Up
			joint_min = 0.0;
			joint_max = 5.9;
			servo_min = 10.0;
			servo_max = 122.0;
			joint_home = joint_min;
			break;
		case 17:	// Left Foot Forward
			joint_min = -28.0;
			joint_max = 52.0;
			servo_min = 24.0;
			servo_max = 150.0;
			joint_home = 0.5 * (joint_min + joint_max);
			break;
		case 18:	// Base Turn
			joint_min = -335.0;
			joint_max = 340.0;
			servo_min = -335.0;
			servo_max =  340.0;
			joint_home = 0.5 * (joint_min + joint_max);
			break;
		default: break;
	}
    
    int dir = (joint_min < joint_max) ? 1 : -1;
    if (dir == 1)
    {
		g_params_res.min[i] = DTOR(joint_min);
		g_params_res.max[i] = DTOR(joint_max);
	}
	else
    {
		g_params_res.min[i] = DTOR(joint_max);
		g_params_res.max[i] = DTOR(joint_min);
	}
    g_params_res.curr[i] = DTOR(joint_home);
  }
  
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
  
  // joint parameters
  double joint_min = 0.0;
  double joint_max = 0.0;
  double joint_home = 0.0;
  double joint_radius = 0.0;

  // servo parameters
  double servo_min = 0.0;
  double servo_max = 0.0;
  double servo_radius = 0.0;
  
	switch (id)
	{
		case  0:	// Mouth
			joint_min = 34.0;
			joint_max = 0.0;
			joint_home = joint_min;
			joint_radius = 2.2;
			
			servo_min = 80.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case  1:	// Head Nod
			joint_min = 96.0;
			joint_max = 40.0;
			joint_home = 90.0;
			joint_radius = 1.4;
			
			servo_min = 51.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case  2:	// Head Turn
			joint_min = -35.0;
			joint_max = 38.0;
			joint_home = 0.5 * (joint_min + joint_max);
			joint_radius = 1.8;
			
			servo_min = 48.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case  3:	// Right Arm Forw
			joint_min = 12.0;
			joint_max = 96.0;
			joint_home = joint_min;
			joint_radius = 1.8;
			
			servo_min = 28.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case  4:	// Right Arm Out
			joint_min = 15.0;
			joint_max = 91.0;
			joint_home = joint_min;
			joint_radius = 2.0;
			
			servo_min = 36.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case  5:	// Right Elbow
			joint_min = 12.0;
			joint_max = 105.0;
			joint_home = joint_min;
			joint_radius = 1.5;
			
			servo_min = 23.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case  6:	// Left Arm Forw
			joint_min = 13.0;
			joint_max = 101.0;
			joint_home = joint_min;
			joint_radius = 1.8;
			
			servo_min = 24.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case  7:	// Left Arm Out
			joint_min = 13.0;
			joint_max = 88.0;
			joint_home = joint_min;
			joint_radius = 2.0;
			
			servo_min = 22.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case  8:	// Left Elbow
			joint_min = 0.0;
			joint_max = 104.0;
			joint_home = joint_min;
			joint_radius = 1.5;
			
			servo_min = 22.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case  9:	// Right Wrist
			joint_min = -86.0;
			joint_max = 55.0;
			joint_home = 0.5 * (joint_min + joint_max);
			joint_radius = 1.0;
			
			servo_min = 38.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case 10:	// Left Wrist
			joint_min = -90.0;
			joint_max = 45.0;
			joint_home = 0.5 * (joint_min + joint_max);
			joint_radius = 1.0;
			
			servo_min = 28.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case 11:	// Body Forw
			joint_min = 19.0;
			joint_max = 85.0;
			joint_home = joint_min;
			joint_radius = 2.0;
			
			servo_min = 30.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case 12:	// Eyelids
			joint_min = 130.0;
			joint_max = 0.0;
			joint_home = 0.5 * (joint_min + joint_max);
			joint_radius = 0.7;
			
			servo_min = 62.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case 13:	// Eye Left/Right
			joint_min = -45.0;//-35.0;
			joint_max = 45.0;//44.0;
			joint_home = 0.5 * (joint_min + joint_max);
			joint_radius = 2.0;
			
			servo_min = 46.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case 14:	// Right Foot Up
			joint_min = 0.0;
			joint_max = 5.9;
			joint_home = joint_min;
			joint_radius = 0.0;
			
			servo_min = 10.0;
			servo_max = 122.0;
			servo_radius = 4.5;
			break;
		case 15:	// Right Foot Forward
			joint_min = 35.0;
			joint_max = -52.0;
			joint_home = 0.5 * (joint_min + joint_max);
			joint_radius = 1.6;
			
			servo_min = 18.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case 16:	// Left Foot Up
			joint_min = 0.0;
			joint_max = 5.9;
			joint_home = joint_min;
			joint_radius = 0.0;
			
			servo_min = 10.0;
			servo_max = 122.0;
			servo_radius = 4.5;
			break;
		case 17:	// Left Foot Forward
			joint_min = -28.0;
			joint_max = 52.0;
			joint_home = 0.5 * (joint_min + joint_max);
			joint_radius = 1.6;
			
			servo_min = 24.0;
			servo_max = 150.0;
			servo_radius = 1.5;
			break;
		case 18:	// Base Turn
			joint_min = -335.0;
			joint_max = 340.0;
			joint_home = 0.5 * (joint_min + joint_max);
			joint_radius = 0.0;
			
			servo_min = -335.0;
			servo_max =  340.0;
			servo_radius = 0.0;
			break;
		default: break;
	}
  double dir = (joint_min < joint_max) ? 1.0 : -1.0;

  double joint_angle = RTOD(angle);
  double servo_angle = 0.0;
  if ((id <= 13) || (id == 17))
		servo_angle = DTOR(servo_max) - acos(1.0 - dir * (DTOR(joint_max) - DTOR(joint_angle)) * joint_radius / servo_radius);
	else if (id == 18)
	  servo_angle = DTOR(joint_angle);
	else
	  servo_angle = acos(1.0 - DTOR(joint_angle) / servo_radius);
	  
	ROS_INFO("Moving joint[%d] (servo[%d, %d]) to %.2f (%.2f)", id, device, channel, joint_angle, servo_angle);
  if (!g_sparky.setServoAngleTarget(device, channel, servo_angle))
  {
    ROS_ERROR("Error moving joint[%d] (servo[%d, %d] to %.2f (%.2f)!", id, device, channel, joint_angle, servo_angle);
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
  pololu::maestro::ServoLimits servo_limits(900, 2100);
  pololu::maestro::ServoAnglePair min_limit(1100, DTOR(0.0));
  pololu::maestro::ServoAnglePair max_limit(2100, DTOR(150.0));
  pololu::maestro::ServoAngleLimits angle_limits(min_limit, max_limit);
  pololu::maestro::ServoLimits base_servo_limits(754, 2254);
  pololu::maestro::ServoAnglePair base_min_limit(754, DTOR(-335.0));
  pololu::maestro::ServoAnglePair base_max_limit(2254, DTOR(340.0));
  pololu::maestro::ServoAngleLimits base_angle_limits(base_min_limit, base_max_limit);
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
      joint_success = g_sparky.setServoAngleLimits(device, channel, base_angle_limits) && joint_success;
    }
    else
    {
      joint_success = g_sparky.setServoLimits(device, channel, servo_limits) && joint_success;
      joint_success = g_sparky.setServoAngleLimits(device, channel, angle_limits) && joint_success;
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
