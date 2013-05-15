/**
 * bandit_smooth.cpp
 *
 * This file implements a node that receives bandit joint array 
 * commands and smoothly interpolates the joint positions to the 
 * target positions by enforcing a maximum joint velocity constraint. 
 *
 * Authors: Juan Fasola, David Feil-Seifer  and Dan Ho
 * Date Written: 9/6/10
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <stdio.h>

/* Degrees/radians macro */
#define DTOR(a) ((a) * M_PI/180.0)
#define RTOD(a) ((a) * 180.0/M_PI)

/* Number of bandit joints */

/* Size of publisher/subscriber message buffer */
const unsigned int MESSAGE_BUFLEN = 5;

/* Maximum joint velocity in degrees/second */
const double MAX_VELOCITY = DTOR(70);

/* Maximum joint acceleration in degrees/second^2 */
const double MAX_ACCELERATION = DTOR(130);

/* Array of current joint angles */
std::map<std::string,double> current_joints;

/* Array of target joint angles */
std::map<std::string,double> target_joints;

/* Array of current velocity of joints */
std::map<std::string,double> velocities;

std::vector<std::string> joint_names;

/* Use of joint dead zone */
const bool USE_DEAD_ZONE = true;

/* Dead zone threshold in degrees */
const double ZONE_THRESHOLD = DTOR(5);


/**
 * Callback function for joint array target requests.
 */
void targetRequestCB(const sensor_msgs::JointStateConstPtr& j){
  printf("Received joint target message!\n");

  /* Address all joint target positions */
  for(unsigned int i=0; i<j->position.size(); i++){
    /* Get target information for joint */
   	std::string name = j->name[i];
		int found = -1;
		printf( "joint_names: %d, %s", joint_names.size(), name.c_str() );
		for( int ii = 0; ii < joint_names.size(); ii++ )
		{
			if( joint_names[ii] == name ) found = ii;
		}

		if( found < 0  ) 
		{
			joint_names.push_back( name );
			current_joints[name] = 0;
			velocities[name] = 0;
			target_joints[name] = 0;
		}


    double angle = j->position[i];

    /* Set target joint angle if valid */

    if(!USE_DEAD_ZONE || (fabs(angle - current_joints[name]) >= ZONE_THRESHOLD)){
			target_joints[name] = angle;
			velocities[name] = 0;
    }

    printf("   updating joint %s to %f\n",name.c_str(),angle);
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
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_cmd", MESSAGE_BUFLEN);

  /* Subscribe to topic to receive target joint angles */
  ros::Subscriber target_sub = n.subscribe("joint_smooth", MESSAGE_BUFLEN, targetRequestCB);

  /* Set loop rate */
  ros::Rate loop_rate(30);

  /* Declare interpolation variables */
  double cur_time, diff_time, vel;
  double last_time = ros::Time::now().toSec();
  bool last_moving, moving = false;

	joint_names.clear();

  printf("Running bandit_smooth node...\n\n");

  /* Loop and send updated movement commands for each joint */
  while (ros::ok())
  {
    /* Build joint array message to send to motion control */
    sensor_msgs::JointState msg;

    /* Update time variables for velocity control */
    cur_time = ros::Time::now().toSec();
    diff_time = (cur_time - last_time);
    last_time = cur_time;

    /* Keep track of robot arm update status */
    last_moving = moving;
    moving = false;

    /* Update joint positions according to target positions */
    for(unsigned i = 0; i<joint_names.size(); i++){

      double c = current_joints[joint_names[i]];  // Current joint position
      double g = target_joints[joint_names[i]];   // Target joint position

      /* Current joint position not at target position */
      if(fabs(g - c) > 1e-6){
				/* Calculate distance from target */
				double dist = fabs(g - c);

				/* Update velocity of joint */
				velocities[joint_names[i]] = velocities[joint_names[i]] + MAX_ACCELERATION * diff_time;
				if(velocities[joint_names[i]] > MAX_VELOCITY){
				  velocities[joint_names[i]] = MAX_VELOCITY;
				}
				vel = diff_time * velocities[joint_names[i]];

				/* Move to target */
				if(g > c){
				  if(dist > vel){
				    c+=vel;
				  }else{
				    c+=dist;
				  }
				}else{
				  if(dist > vel){
				    c-=vel;
				  }else{
			    c-=dist;
	  			}
				}

				printf( "%s: %0.2f ==> %0.2f\n", joint_names[i].c_str(), c, g );

				/* Update internal position of joint */
				current_joints[joint_names[i]] = c;

				/* Add updated joint position to joint state message */
				msg.name.push_back(joint_names[i]);
				msg.position.push_back(c);
				msg.velocity.push_back(0);
				msg.effort.push_back(0);
				moving = true;
      }
    }

    /* Joint position to be modified */
    if(moving){
      /* Publish joint array message to motion control */
      joint_pub.publish(msg);
      if(!last_moving)
				printf("moving... cur time: %f\n",cur_time);
    } else if(last_moving){
      printf("Done moving! cur time: %f\n\n",cur_time);
    }

    /* Spin necessary for callbacks */
    ros::spinOnce();

    /* Sleep to reach desired frame rate */
    loop_rate.sleep();
  }

  return 0;
}


