// preprocessor directives
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>
#include <pr2_boxer/PR2LeftArmConfig.h>
#include <pr2_boxer/PR2RightArmConfig.h>
#include <angles/angles.h>
#include <string>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
using namespace std;

//
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajActionClient;

//
class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajActionClient* traj_client_;
  std::vector<std::string> joint_names_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm(std::string path, std::vector<std::string> joint_names = std::vector<std::string>()) :
      joint_names_(joint_names)
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajActionClient(path, true); //"r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while (!traj_client_->waitForServer(ros::Duration(5.0)))
      ROS_INFO("Waiting for the joint_trajectory_action server");
  } // RobotArm()

  //! Clean up the action client
  ~RobotArm()
  {
    if (traj_client_ != NULL)
    {
      delete traj_client_;
      traj_client_ = NULL;
    }
  } // ~RobotArm()

  //! Set the joint names
  bool setJointNames(std::vector<std::string> joint_names)
  {
    if (joint_names.empty())
      return false;
    joint_names_ = joint_names;
    return true;
  } // setJointNames(std::vector<std::string>)

  //! Get the joint names
  std::vector<std::string> getJointNames()
  {
    return joint_names_;
  } // getJointNames()

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  } // getState()

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal, double wait_for = 0.0)
  {
    // When to start the trajectory: 'wait_for' from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(wait_for);
    traj_client_->sendGoal(goal);
  } // startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal, double)

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
   as a single trajectory. Alternatively, each of these waypoints could
   be in its own trajectory - a trajectory can have one or more waypoints
   depending on the desired application.
   */
  pr2_controllers_msgs::JointTrajectoryGoal trajStance()
  {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names = getJointNames();

    // We will have one waypoint in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point

    // Positions
    int waypoint = 0;
    goal.trajectory.points[waypoint].positions.resize(7);
    goal.trajectory.points[waypoint].positions[0] = angles::from_degrees(10.0);
    goal.trajectory.points[waypoint].positions[1] = angles::from_degrees(20.0);
    goal.trajectory.points[waypoint].positions[2] = angles::from_degrees(0.0);
    goal.trajectory.points[waypoint].positions[3] = angles::from_degrees(-110.0);
    goal.trajectory.points[waypoint].positions[4] = angles::from_degrees(-90.0);
    goal.trajectory.points[waypoint].positions[5] = angles::from_degrees(0.0);
    goal.trajectory.points[waypoint].positions[6] = angles::from_degrees(180.0);

    // Velocities
    goal.trajectory.points[waypoint].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[waypoint].velocities[j] = 0.0;
    }

    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[waypoint].time_from_start = ros::Duration(0.01);
    /*
     // Second trajectory point
     // Positions
     waypoint += 1;
     goal.trajectory.points[waypoint].positions.resize(7);
     goal.trajectory.points[waypoint].positions[0] = -0.3;
     goal.trajectory.points[waypoint].positions[1] = 0.2;
     goal.trajectory.points[waypoint].positions[2] = -0.1;
     goal.trajectory.points[waypoint].positions[3] = -1.2;
     goal.trajectory.points[waypoint].positions[4] = 1.5;
     goal.trajectory.points[waypoint].positions[5] = -0.3;
     goal.trajectory.points[waypoint].positions[6] = 0.5;
     // Velocities
     goal.trajectory.points[waypoint].velocities.resize(7);
     for (size_t j = 0; j < 7; ++j)
     {
     goal.trajectory.points[waypoint].velocities[j] = 0.0;
     }
     // To be reached 2 seconds after starting along the trajectory
     goal.trajectory.points[waypoint].time_from_start = ros::Duration(2.0);
     */

    //we are done; return the goal
    return goal;
  } // trajIdle()

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
   as a single trajectory. Alternatively, each of these waypoints could
   be in its own trajectory - a trajectory can have one or more waypoints
   depending on the desired application.
   */

  pr2_controllers_msgs::JointTrajectoryGoal trajCross()
  {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names = getJointNames();

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point

    // Positions
    int waypoint = 0;
    goal.trajectory.points[waypoint].positions.resize(7);
    goal.trajectory.points[waypoint].positions[0] = 0.1;
    goal.trajectory.points[waypoint].positions[1] = 0.0;
    goal.trajectory.points[waypoint].positions[2] = -0.7;
    goal.trajectory.points[waypoint].positions[3] = -0.1;
    goal.trajectory.points[waypoint].positions[4] = 3.1415;
    goal.trajectory.points[waypoint].positions[5] = 0.0;
    goal.trajectory.points[waypoint].positions[6] = 0.0;
    // Velocities
    goal.trajectory.points[waypoint].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[waypoint].velocities[j] = 0.0;
    }

    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[waypoint].time_from_start = ros::Duration(0.01);
    /*
     // Second trajectory point
     // Positions
     waypoint += 1;
     goal.trajectory.points[waypoint].positions.resize(7);
     goal.trajectory.points[waypoint].positions[0] = -0.3;
     goal.trajectory.points[waypoint].positions[1] = 0.2;
     goal.trajectory.points[waypoint].positions[2] = -0.1;
     goal.trajectory.points[waypoint].positions[3] = -1.2;
     goal.trajectory.points[waypoint].positions[4] = 1.5;
     goal.trajectory.points[waypoint].positions[5] = -0.3;
     goal.trajectory.points[waypoint].positions[6] = 0.5;
     // Velocities
     goal.trajectory.points[waypoint].velocities.resize(7);
     for (size_t j = 0; j < 7; ++j)
     {
     goal.trajectory.points[waypoint].velocities[j] = 0.0;
     }
     // To be reached 2 seconds after starting along the trajectory
     goal.trajectory.points[waypoint].time_from_start = ros::Duration(2.0);
     */

    //we are done; return the goal
    return goal;
  } // trajCross()
};
// RobotArm

#define L_ARM ( 1)
#define R_ARM (-1)

// global variables for right arm
std::vector<std::string> g_l_arm_joint_names;
std::vector<double> g_l_arm_goal_points;
double g_l_arm_time_from_start = 0.0;
bool g_l_arm_send_goal = false;

// global variables for right arm
std::vector<std::string> g_r_arm_joint_names;
std::vector<double> g_r_arm_goal_points;
double g_r_arm_time_from_start = 0.0;
bool g_r_arm_send_goal = false;

// function prototypes
pr2_controllers_msgs::JointTrajectoryGoal playMotion(int arm, double tstep, double tstart, char* fname);
pr2_controllers_msgs::JointTrajectoryGoal getTrajectoryStance(int arm);
pr2_controllers_msgs::JointTrajectoryGoal getTrajectoryJab(int arm);
pr2_controllers_msgs::JointTrajectoryGoal getTrajectoryCross(int arm);
pr2_controllers_msgs::JointTrajectoryGoal getTrajectoryHook(int arm);
pr2_controllers_msgs::JointTrajectoryGoal getTrajectoryUppercut(int arm);
pr2_controllers_msgs::JointTrajectoryGoal getTrajectoryConfig(int arm);
pr2_controllers_msgs::JointTrajectoryGoal getTrajectory(std::vector<std::string> joint_names,
                                                        std::vector<double> goal_points, double time_from_start = 0.0);
void cbLeftArmConfig(pr2_boxer::PR2LeftArmConfig &config, uint32_t level);
void cbRightArmConfig(pr2_boxer::PR2RightArmConfig &config, uint32_t level);

// executes main program code
int main(int argc, char** argv)
{
  double tstep = atof(argv[1]);
  double tstart = atof(argv[2]);
  double runtime = atof(argv[3]);
  char * fname = argv[4];

  printf("inputs were: %f, %f, %f",tstep,tstart,runtime);
  // Init the ROS node
  ros::init(argc, argv, "pr2_boxer");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_l_arm(nh, "l_arm");
  ros::NodeHandle nh_r_arm(nh, "r_arm");

  // initialize parameters for left arm
  nh_l_arm.setParam("l_shoulder_pan_joint", 0.0);
  nh_l_arm.setParam("l_shoulder_lift_joint", 0.0);
  nh_l_arm.setParam("l_upper_arm_roll_joint", 0.0);
  nh_l_arm.setParam("l_elbow_flex_joint", 0.0);
  nh_l_arm.setParam("l_forearm_roll_joint", 0.0);
  nh_l_arm.setParam("l_wrist_flex_joint", 0.0);
  nh_l_arm.setParam("l_wrist_roll_joint", 0.0);

  // initialize parameters for right arm
  nh_r_arm.setParam("r_shoulder_pan_joint", 0.0);
  nh_r_arm.setParam("r_shoulder_lift_joint", 0.0);
  nh_r_arm.setParam("r_upper_arm_roll_joint", 0.0);
  nh_r_arm.setParam("r_elbow_flex_joint", 0.0);
  nh_r_arm.setParam("r_forearm_roll_joint", 0.0);
  nh_r_arm.setParam("r_wrist_flex_joint", 0.0);
  nh_r_arm.setParam("r_wrist_roll_joint", 0.0);

  // initialize dynamic reconfigure parameter server for left arm
  dynamic_reconfigure::Server<pr2_boxer::PR2LeftArmConfig> srv_l_arm_reconfig(nh_l_arm);
  dynamic_reconfigure::Server<pr2_boxer::PR2LeftArmConfig>::CallbackType cb_l_arm_reconfig;
  cb_l_arm_reconfig = boost::bind(&cbLeftArmConfig, _1, _2);
  srv_l_arm_reconfig.setCallback(cb_l_arm_reconfig);

  // initialize dynamic reconfigure parameter server for right arm
  dynamic_reconfigure::Server<pr2_boxer::PR2RightArmConfig> srv_r_arm_reconfig(nh_r_arm);
  dynamic_reconfigure::Server<pr2_boxer::PR2RightArmConfig>::CallbackType cb_r_arm_reconfig;
  cb_r_arm_reconfig = boost::bind(&cbRightArmConfig, _1, _2);
  srv_r_arm_reconfig.setCallback(cb_r_arm_reconfig);

  // the joint names for the left arm, which apply to all waypoints
  g_l_arm_joint_names.push_back("l_shoulder_pan_joint");
  g_l_arm_joint_names.push_back("l_shoulder_lift_joint");
  g_l_arm_joint_names.push_back("l_upper_arm_roll_joint");
  g_l_arm_joint_names.push_back("l_elbow_flex_joint");
  g_l_arm_joint_names.push_back("l_forearm_roll_joint");
  g_l_arm_joint_names.push_back("l_wrist_flex_joint");
  g_l_arm_joint_names.push_back("l_wrist_roll_joint");

  // the joint names for the right arm, which apply to all waypoints
  g_r_arm_joint_names.push_back("r_shoulder_pan_joint");
  g_r_arm_joint_names.push_back("r_shoulder_lift_joint");
  g_r_arm_joint_names.push_back("r_upper_arm_roll_joint");
  g_r_arm_joint_names.push_back("r_elbow_flex_joint");
  g_r_arm_joint_names.push_back("r_forearm_roll_joint");
  g_r_arm_joint_names.push_back("r_wrist_flex_joint");
  g_r_arm_joint_names.push_back("r_wrist_roll_joint");

  RobotArm l_arm("l_arm_controller/joint_trajectory_action", g_l_arm_joint_names);
  RobotArm r_arm("r_arm_controller/joint_trajectory_action", g_r_arm_joint_names);

  
  while (ros::ok())
  {
    printf("L-stance | R-stance\n");
    l_arm.startTrajectory(getTrajectoryStance(L_ARM), 2.0);
    r_arm.startTrajectory(getTrajectoryStance(R_ARM), 2.0);
    while (ros::ok() && ((!l_arm.getState().isDone()) || (!r_arm.getState().isDone())))
        usleep(10000);

    printf("L-stance | R-playMotion\n");
    l_arm.startTrajectory(getTrajectoryStance(L_ARM), 2.0);
    r_arm.startTrajectory(playMotion(R_ARM,tstep,tstart,fname), runtime);
    while (ros::ok() && ((!l_arm.getState().isDone()) || (!r_arm.getState().isDone())))
      usleep(10000);
    
    ros::spinOnce();
  }
} // main(int, char**)

/*              Beginning of Tarik's edits.                   */ 

//
pr2_controllers_msgs::JointTrajectoryGoal playMotion(int arm, double TSTEP, double TSTART, char* fname)
{
  int NUM = 180;
//  int TSTEP = 2;
 
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  goal.trajectory.joint_names = (arm == L_ARM) ? g_l_arm_joint_names : g_r_arm_joint_names;
  goal.trajectory.points.resize(NUM); // number of waypoints

  // Iteratively set waypoints, reading from file:
  ifstream inFile (fname);
                  //"/home/tarik/tarik_retargeter/motionPlayer/plain.txt");
  string line;
  char * pch;
  
  int waypoint = -1;
  double time_from_start = TSTART; // time to get to first pose.
  int counter = 0;
  printf("about to play\n");
  for (int i=0; i<NUM; i++) {
      //waypoint++;
      time_from_start += TSTEP;
      if(inFile.is_open()) {
         if(getline(inFile, line)) {
             //if ((counter++ % 5) != 0) continue;
             ++waypoint;
             printf("file says: %s\n",line.c_str());
             // positions
             goal.trajectory.points[waypoint].positions.resize(7);
             pch = strtok ((char*)line.c_str()," ");
             goal.trajectory.points[waypoint].positions[0] = angles::from_degrees(atof(pch));
             pch = strtok(NULL, " ");
             goal.trajectory.points[waypoint].positions[1] = angles::from_degrees(atof(pch));
             pch = strtok(NULL, " ");
             goal.trajectory.points[waypoint].positions[2] = angles::from_degrees(atof(pch));
             pch = strtok(NULL, " ");
             goal.trajectory.points[waypoint].positions[3] = angles::from_degrees(atof(pch));
             goal.trajectory.points[waypoint].positions[4] = angles::from_degrees(0.0);
             goal.trajectory.points[waypoint].positions[5] = angles::from_degrees(0.0);
             goal.trajectory.points[waypoint].positions[6] = angles::from_degrees(90.0);
         }
         else {
            printf("File ended sooner than expected!\n");
            break;  //file over
         }
      }
      else {
         printf("file error\n");
      }   
       
      printf("positions assigned.\n");
      // velocities
      goal.trajectory.points[waypoint].velocities.resize(7);
      for (size_t i = 0; i < 7; ++i)
        goal.trajectory.points[waypoint].velocities[i] = 0.0;
    
      // To be reached 1 second after starting along the trajectory
      goal.trajectory.points[waypoint].time_from_start = ros::Duration(time_from_start);
  }
  printf("about to return goal\n");
  return goal;
} // getTrajectoryCross(int)
//
/*          End of Tarik's edits            */

pr2_controllers_msgs::JointTrajectoryGoal getTrajectoryStance(int arm)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.joint_names = (arm == L_ARM) ? g_l_arm_joint_names : g_r_arm_joint_names;

  goal.trajectory.points.resize(1); // number of waypoints

  // WAYPOINT 0
  int waypoint = 0;
  double time_from_start = 2.0;

  // positions
  goal.trajectory.points[waypoint].positions.resize(7);
  goal.trajectory.points[waypoint].positions[0] = angles::from_degrees(110.0 * arm);
  goal.trajectory.points[waypoint].positions[1] = angles::from_degrees(75.0);
  goal.trajectory.points[waypoint].positions[2] = angles::from_degrees(110.0 * arm);
  goal.trajectory.points[waypoint].positions[3] = angles::from_degrees(-30.0);
  goal.trajectory.points[waypoint].positions[4] = angles::from_degrees(0.0 * arm);
  goal.trajectory.points[waypoint].positions[5] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[6] = angles::from_degrees(0.0 * arm);

  // velocities
  goal.trajectory.points[waypoint].velocities.resize(7);
  for (size_t i = 0; i < 7; ++i)
    goal.trajectory.points[waypoint].velocities[i] = 0.1;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[waypoint].time_from_start = ros::Duration(time_from_start);

  return goal;
} // getTrajectoryStance(int)

//
pr2_controllers_msgs::JointTrajectoryGoal getTrajectoryJab(int arm)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.joint_names = (arm == L_ARM) ? g_l_arm_joint_names : g_r_arm_joint_names;

  goal.trajectory.points.resize(1); // number of waypoints

  // WAYPOINT 0
  int waypoint = 0;
  double time_from_start = 0.5;

  // positions
  goal.trajectory.points[waypoint].positions.resize(7);
  goal.trajectory.points[waypoint].positions[0] = angles::from_degrees(-10.0 * arm);
  goal.trajectory.points[waypoint].positions[1] = angles::from_degrees(-20.0);
  goal.trajectory.points[waypoint].positions[2] = angles::from_degrees(0.0 * arm);
  goal.trajectory.points[waypoint].positions[3] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[4] = angles::from_degrees(0.0 * arm);
  goal.trajectory.points[waypoint].positions[5] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[6] = angles::from_degrees(180.0 * arm);

  // velocities
  goal.trajectory.points[waypoint].velocities.resize(7);
  for (size_t i = 0; i < 7; ++i)
    goal.trajectory.points[waypoint].velocities[i] = 0.0;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[waypoint].time_from_start = ros::Duration(time_from_start);

  return goal;
} // getTrajectoryJab(int)

//
pr2_controllers_msgs::JointTrajectoryGoal getTrajectoryCross(int arm)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.joint_names = (arm == L_ARM) ? g_l_arm_joint_names : g_r_arm_joint_names;

  goal.trajectory.points.resize(2); // number of waypoints

  // WAYPOINT 0
  int waypoint = 0;
  double time_from_start = 1.0;

  // positions
  goal.trajectory.points[waypoint].positions.resize(7);
  goal.trajectory.points[waypoint].positions[0] = angles::from_degrees(45.0 * arm);
  goal.trajectory.points[waypoint].positions[1] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[2] = angles::from_degrees(90.0 * arm);
  goal.trajectory.points[waypoint].positions[3] = angles::from_degrees(-40.0);
  goal.trajectory.points[waypoint].positions[4] = angles::from_degrees(0.0 * arm);
  goal.trajectory.points[waypoint].positions[5] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[6] = angles::from_degrees(90.0 * arm);

  // velocities
  goal.trajectory.points[waypoint].velocities.resize(7);
  for (size_t i = 0; i < 7; ++i)
    goal.trajectory.points[waypoint].velocities[i] = 0.0;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[waypoint].time_from_start = ros::Duration(time_from_start);

  // WAYPOINT 1
  ++waypoint;
  time_from_start += 0.1;

  // positions
  goal.trajectory.points[waypoint].positions.resize(7);
  goal.trajectory.points[waypoint].positions[0] = angles::from_degrees(0.0 * arm);
  goal.trajectory.points[waypoint].positions[1] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[2] = angles::from_degrees(90.0 * arm);
  goal.trajectory.points[waypoint].positions[3] = angles::from_degrees(-40.0);
  goal.trajectory.points[waypoint].positions[4] = angles::from_degrees(0.0 * arm);
  goal.trajectory.points[waypoint].positions[5] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[6] = angles::from_degrees(90.0 * arm);

  // velocities
  goal.trajectory.points[waypoint].velocities.resize(7);
  for (size_t i = 0; i < 7; ++i)
    goal.trajectory.points[waypoint].velocities[i] = 0.0;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[waypoint].time_from_start = ros::Duration(time_from_start += 0.5);

  return goal;
} // getTrajectoryCross(int)

//
pr2_controllers_msgs::JointTrajectoryGoal getTrajectoryHook(int arm)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.joint_names = (arm == L_ARM) ? g_l_arm_joint_names : g_r_arm_joint_names;

  goal.trajectory.points.resize(2); // number of waypoints

  // WAYPOINT 0
  int waypoint = 0;
  double time_from_start = 1.0;

  // positions
  goal.trajectory.points[waypoint].positions.resize(7);
  goal.trajectory.points[waypoint].positions[0] = angles::from_degrees(90.0 * arm);
  goal.trajectory.points[waypoint].positions[1] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[2] = angles::from_degrees(90.0 * arm);
  goal.trajectory.points[waypoint].positions[3] = angles::from_degrees(-90.0);
  goal.trajectory.points[waypoint].positions[4] = angles::from_degrees(0.0 * arm);
  goal.trajectory.points[waypoint].positions[5] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[6] = angles::from_degrees(90.0 * arm);

  // velocities
  goal.trajectory.points[waypoint].velocities.resize(7);
  for (size_t i = 0; i < 7; ++i)
    goal.trajectory.points[waypoint].velocities[i] = 0.0;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[waypoint].time_from_start = ros::Duration(time_from_start);

  // WAYPOINT 1
  ++waypoint;
  time_from_start += 0.1;

  // positions
  goal.trajectory.points[waypoint].positions.resize(7);
  goal.trajectory.points[waypoint].positions[0] = angles::from_degrees(0.0 * arm);
  goal.trajectory.points[waypoint].positions[1] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[2] = angles::from_degrees(90.0 * arm);
  goal.trajectory.points[waypoint].positions[3] = angles::from_degrees(-60.0);
  goal.trajectory.points[waypoint].positions[4] = angles::from_degrees(0.0 * arm);
  goal.trajectory.points[waypoint].positions[5] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[6] = angles::from_degrees(90.0 * arm);

  // velocities
  goal.trajectory.points[waypoint].velocities.resize(7);
  for (size_t i = 0; i < 7; ++i)
    goal.trajectory.points[waypoint].velocities[i] = 0.0;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[waypoint].time_from_start = ros::Duration(time_from_start);

  return goal;
} // getTrajectoryHook(int)

//
pr2_controllers_msgs::JointTrajectoryGoal getTrajectoryUppercut(int arm)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.joint_names = (arm == L_ARM) ? g_l_arm_joint_names : g_r_arm_joint_names;

  goal.trajectory.points.resize(2); // number of waypoints

  // WAYPOINT 0
  int waypoint = 0;
  double time_from_start = 0.5;

  // positions
  goal.trajectory.points[waypoint].positions.resize(7);
  goal.trajectory.points[waypoint].positions[0] = angles::from_degrees(-10.0 * arm);
  goal.trajectory.points[waypoint].positions[1] = angles::from_degrees(60.0);
  goal.trajectory.points[waypoint].positions[2] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[3] = angles::from_degrees(-130.0);
  goal.trajectory.points[waypoint].positions[4] = angles::from_degrees(0.0 * arm);
  goal.trajectory.points[waypoint].positions[5] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[6] = angles::from_degrees(0.0 * arm);

  // velocities
  goal.trajectory.points[waypoint].velocities.resize(7);
  for (size_t i = 0; i < 7; ++i)
    goal.trajectory.points[waypoint].velocities[i] = 0.0;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[waypoint].time_from_start = ros::Duration(time_from_start);

  // WAYPOINT 1
  ++waypoint;
  time_from_start += 0.1;

  // positions
  goal.trajectory.points[waypoint].positions.resize(7);
  goal.trajectory.points[waypoint].positions[0] = angles::from_degrees(-5.0 * arm);
  goal.trajectory.points[waypoint].positions[1] = angles::from_degrees(-30.0);
  goal.trajectory.points[waypoint].positions[2] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[3] = angles::from_degrees(-90.0);
  goal.trajectory.points[waypoint].positions[4] = angles::from_degrees(0.0 * arm);
  goal.trajectory.points[waypoint].positions[5] = angles::from_degrees(0.0);
  goal.trajectory.points[waypoint].positions[6] = angles::from_degrees(0.0 * arm);

  // velocities
  goal.trajectory.points[waypoint].velocities.resize(7);
  for (size_t i = 0; i < 7; ++i)
    goal.trajectory.points[waypoint].velocities[i] = 0.0;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[waypoint].time_from_start = ros::Duration(time_from_start);

  return goal;
} // getTrajectoryUppercut(int)

//
pr2_controllers_msgs::JointTrajectoryGoal getTrajectoryConfig(int arm)
{
  std::vector<std::string> joint_names;
  std::vector<double> goal_points;
  double time_from_start = 0.0;

  if (arm == L_ARM)
  {
    joint_names = g_l_arm_joint_names;
    goal_points = g_l_arm_goal_points;
    time_from_start = g_l_arm_time_from_start;
  }
  else if (arm == R_ARM)
  {
    joint_names = g_r_arm_joint_names;
    goal_points = g_r_arm_goal_points;
    time_from_start = g_r_arm_time_from_start;
  }
  else
    return pr2_controllers_msgs::JointTrajectoryGoal();

  return getTrajectory(joint_names, goal_points, time_from_start);
} // getTrajectoryConfig(int)

//
pr2_controllers_msgs::JointTrajectoryGoal getTrajectory(std::vector<std::string> joint_names,
                                                        std::vector<double> goal_points, double time_from_start)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.joint_names = joint_names;

  goal.trajectory.points.resize(1); // number of waypoints

  goal.trajectory.points[0].positions.resize(goal_points.size());
  goal.trajectory.points[0].velocities.resize(goal_points.size());
  for (int i = 0, n = goal_points.size(); i < n; ++i)
  {
    goal.trajectory.points[0].positions[i] = goal_points[i];
    goal.trajectory.points[0].velocities[i] = 0.0;
  }

  goal.trajectory.points[0].time_from_start = ros::Duration(time_from_start);

  return goal;
} // getTrajectory(std::vector<std::string>, std::vector<double>, double)

//
void cbLeftArmConfig(pr2_boxer::PR2LeftArmConfig &config, uint32_t level)
{
  g_l_arm_goal_points.clear();
  g_l_arm_goal_points.resize(7);
  g_l_arm_goal_points[0] = angles::from_degrees(config.l_shoulder_pan_joint);
  g_l_arm_goal_points[1] = angles::from_degrees(config.l_shoulder_lift_joint);
  g_l_arm_goal_points[2] = angles::from_degrees(config.l_upper_arm_roll_joint);
  g_l_arm_goal_points[3] = angles::from_degrees(config.l_elbow_flex_joint);
  g_l_arm_goal_points[4] = angles::from_degrees(config.l_forearm_roll_joint);
  g_l_arm_goal_points[5] = angles::from_degrees(config.l_wrist_flex_joint);
  g_l_arm_goal_points[6] = angles::from_degrees(config.l_wrist_roll_joint);

  g_l_arm_time_from_start = config.time_from_start;

  g_l_arm_send_goal = true;
} // cbLeftArmConfig(pr2_boxer::PR2LeftArmConfig &, uint32_t)

//
void cbRightArmConfig(pr2_boxer::PR2RightArmConfig &config, uint32_t level)
{
  g_r_arm_goal_points.clear();
  g_r_arm_goal_points.resize(7);
  g_r_arm_goal_points[0] = angles::from_degrees(config.r_shoulder_pan_joint);
  g_r_arm_goal_points[1] = angles::from_degrees(config.r_shoulder_lift_joint);
  g_r_arm_goal_points[2] = angles::from_degrees(config.r_upper_arm_roll_joint);
  g_r_arm_goal_points[3] = angles::from_degrees(config.r_elbow_flex_joint);
  g_r_arm_goal_points[4] = angles::from_degrees(config.r_forearm_roll_joint);
  g_r_arm_goal_points[5] = angles::from_degrees(config.r_wrist_flex_joint);
  g_r_arm_goal_points[6] = angles::from_degrees(config.r_wrist_roll_joint);

  g_r_arm_time_from_start = config.time_from_start;

  g_r_arm_send_goal = true;
} // cbRightArmConfig(pr2_boxer::PR2RightArmConfig &, uint32_t)
