/**
 * bandit_smooth.cpp
 *
 * This file implements a node that receives bandit joint array 
 * commands and smoothly interpolates the joint positions to the 
 * target positions by enforcing a maximum joint velocity constraint. 
 *
 * Authors: Juan Fasola and Dan Ho
 * Date Written: 9/6/10
 */

#include "ros/ros.h"
#include "bandit_msgs/JointArray.h"
#include <stdio.h>

/* Degrees/radians macro */
#define DTOR(a) ((a) * M_PI/180.0)
#define RTOD(a) ((a) * 180.0/M_PI)

/* Number of bandit joints */
const int NUM_JOINTS = 19;

/* Size of publisher/subscriber message buffer */
const unsigned int MESSAGE_BUFLEN = 5;

/* Maximum joint velocity in degrees/second */
const double MAX_VELOCITY = DTOR(50);

/* Maximum joint acceleration in degrees/second^2 */
const double MAX_ACCE = DTOR(10);

/* Array of current joint angles */
double current_joints[NUM_JOINTS];

/* Array of target joint angles */
double target_joints[NUM_JOINTS];

/**
 * Callback function for joint array target requests.
 */
void targetRequestCB(const bandit_msgs::JointArrayConstPtr& j){
  printf("Received joint target message!\n");

  /* Address all joint target positions */
  for(unsigned int i=0; i<j->joints.size(); i++){
    /* Get target information for joint */
    int id = j->joints[i].id;
    double angle = j->joints[i].angle;

    /* Set target joint angle if valid */
    if(id>=0 && id<NUM_JOINTS){
      if(fabs(current_joints[id] - angle) > DTOR(5))
        target_joints[id] = angle;
      else
        target_joints[id] = current_joints[id];
   
    }
    printf("   updating joint %d to %f\n",id,angle);
  }
}

/**
 * This Node splits up a bandit joint movement command into multiple commands
 * to interpolate smoothly to the desired position, according to the pre-set
 * maximum joint velocity.
 */
int main(int argc, char **argv)
{
  /* Initialize ROS */
  ros::init(argc, argv, "bandit_smooth");
  ros::NodeHandle n;

  /* Setup publisher to communicate with motion control node */
  ros::Publisher joint_pub = n.advertise<bandit_msgs::JointArray>("joint_cmd", MESSAGE_BUFLEN);

  /* Subscribe to topic to receive target joint angles */
  ros::Subscriber target_sub = n.subscribe("joint_smooth", MESSAGE_BUFLEN, targetRequestCB);

  /* Initialize joint arrays to default position */
  int i;
  for(i=0; i<NUM_JOINTS; i++){
    current_joints[i] = 0;
    target_joints[i] = 0;
  }

  /* Set loop rate */
  ros::Rate loop_rate(30);

  /* Declare interpolation variables */
  double cur_time, diff_time, vel;
  double last_time = ros::Time::now().toSec();
  bool last_moving, moving = false;

  printf("Running bandit_smooth node...\n\n");

  /* Loop and send updated movement commands for each joint */
  while (ros::ok())
  {
    /* Build joint array message to send to motion control */
    bandit_msgs::JointArray msg;

    /* Update time variables for velocity control */
    cur_time = ros::Time::now().toSec();
    diff_time = (cur_time - last_time);
    last_time = cur_time;
    vel = (0.5 * MAX_ACCE * diff_time*diff_time) + diff_time* MAX_VELOCITY; //vel = diff_time * MAX_VELOCITY;

    /* Keep track of robot arm update status */
    last_moving = moving;
    moving = false;

    /* Update joint positions according to target positions */
    for(i=0; i<NUM_JOINTS; i++){
      double c = current_joints[i];  // Current joint position
      double g = target_joints[i];   // Target joint position

      /*Current joint position not at target position */
      if(fabs(g - c) > 1e-6){
	/* Calculate distance from target */
	double dist = fabs(g - c);

	/* Move to target */
	if(g > c){
	  if(dist > vel)
	    c+=vel;
	  else
	    c+=dist;
	}else{
	  if(dist > vel)
	    c-=vel;
	  else
	    c-=dist;
	}

	/* Update internal position of joint */
	current_joints[i] = c;

	/* Add updated joint position to joint array message */
	bandit_msgs::Joint j;
	j.id = i;
	j.angle = c;
	msg.joints.push_back(j);
	moving = true;
      }
    }

    /* Joint position to be modified */
    if(moving){
      /* Publish joint array message to motion control */
      joint_pub.publish(msg);

      if(!last_moving)
	printf("moving... cur time: %f\n",cur_time);
    }else if(last_moving){
      printf("Done moving! cur time: %f\n\n",cur_time);
    }

    /* Spin necessary for callbacks */
    ros::spinOnce();

    /* Sleep to reach desired frame rate */
    loop_rate.sleep();
  }

  return 0;
}


