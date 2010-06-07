// preprocessor directives
#include <ros/ros.h>
#include <joint_msgs/JointArray.h>
#include <joint_msgs/Params.h>
#include <sensor_msgs/JointState.h>
#include <sparky/controller.hh>



// angle (degrees and radians) definitions
#define DTOR(rad) ((rad) * M_PI / 180.0f)
#define RTOD(deg) ((deg) * 180.0f / M_PI)



// global variables
controller::Controller       g_sparky;
joint_msgs::Params::Response g_params_res;



// function prototypes
bool initParams();
bool init_servo_controller();
void shutdown_servo_controller();
bool outputPositions(double* vals, bool* active);



//
bool paramsCallback(joint_msgs::Params::Request  &req,
                    joint_msgs::Params::Response &res)
{
  res = g_params_res;
  return true;
} // paramsCallback(joint_msgs::Params::{Request, Response} &)



//
void jointCallback(const joint_msgs::JointConstPtr &joint)
{
  if (joint->active)
  {
    ROS_INFO("Moving joint[%d] to %.2f", joint->id, joint->angle);
    if (!g_sparky.moveTo(joint->id, joint->angle))
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
      ROS_INFO("Moving joint[%d] to %.2f",
               joints->joints[i].id,
               joints->joints[i].angle);
      if (!g_sparky.moveTo(joints->joints[i].id, joints->joints[i].angle))
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

  if (!init_servo_controller()) return 1;

  // set up ROS publishers
  //ros::Publisher joints_pub = nh.advertise<sensor_msgs::JointState>(
  //                              "joint_states", 1000);
  ros::Publisher joint_pub = nh.advertise<joint_msgs::Joint>("joint_cmd", 1000);
  ros::Publisher joints_pub = nh.advertise<joint_msgs::JointArray>(
                                "joints_cmd", 1000);

  // set up ROS subscribers
  ros::Subscriber joints_sub = nh.subscribe(
                                 "joints_cmd", 1, jointArrayCallback);
  ros::Subscriber joint_sub  = nh.subscribe(
                                 "joint_cmd",  1, jointCallback);

  // set up ROS services
  initParams();
  ros::ServiceServer params_srv = nh.advertiseService(
                                    "sparky_params", paramsCallback);

  //
  while (nh.ok()) ros::spin();

  // exit program
  shutdown_servo_controller();
  return 0;
} // main(int, char**)



bool initParams()
{

  // resize the parameter vectors
  const int n_channels = controller::NChannels;
  g_params_res.name.resize(n_channels);
  g_params_res.id.resize(  n_channels);
  g_params_res.min.resize( n_channels);
  g_params_res.max.resize( n_channels);
  g_params_res.curr.resize(n_channels);

  // set parameter names
  g_params_res.name[ 0] = "Mouth";
  g_params_res.name[ 1] = "Head Nod";
  g_params_res.name[ 2] = "Head Turn";
  g_params_res.name[ 3] = "Right Arm Forw";
  g_params_res.name[ 4] = "Right Arm Out";
  g_params_res.name[ 5] = "Right Elbow";
  g_params_res.name[ 6] = "Left Arm Forw";
  g_params_res.name[ 7] = "Left Arm Out";
  g_params_res.name[ 8] = "Left Elbow";
  g_params_res.name[ 9] = "Right Wrist";
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
    g_params_res.id[i]   = i;
    g_params_res.min[i]  = 0.0f;
    g_params_res.max[i]  = 1.0f;
    g_params_res.curr[i] = 0.5f;
  }

  return true;
} // initParams()



//////////////////////////////////////////////////////////////////////
//
//  init_servo_controller()
//
//  connects to and initializes the Pololu controllers for the minimatronic
//  returns true on success, false on failure
//
//  no arguements
//

bool init_servo_controller()
{

    if ( ! g_sparky.open() ) {
        std::cout << "Unable to open controller, exiting" <<  std::endl;
        return false;
    }

    // success
    return true;
}



//////////////////////////////////////////////////////////////////////
//
//  shutdown_servo_controller()
//
//  shuts down and disconnects the Pololu controllers for the minimatronic
//  returns no value (void)
//
//  no arguements
//

void shutdown_servo_controller()
{
    // move to home (mid stroke)
    g_sparky . moveHome();

    // allow some time to get there before we shutdown
    sleep(1);

    // turn off the controllers
    g_sparky . turnOffAll();

    // close (the close would be done automatically anyway by the dtor)
    g_sparky.close();
}



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
    for (int chan=0; chan<controller::NChannels; ++chan) {
        if (active == 0  || active[chan]) {
            if ( ! g_sparky . moveTo(chan, vals[chan]) ) {
                std::cout << "Error moving channel " << chan+1 <<  std::endl;
                return false;
            }
        }
    }

    // success
    return true;
}

