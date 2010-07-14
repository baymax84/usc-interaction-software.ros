#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "bandit_msgs/JointArray.h"
#include "bandit_msgs/Params.h"
#include "bandit_msgs/CalibrateJoint.h"
#include "bandit_msgs/CalibrateJointAll.h"
#include "math.h"
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;

/*
	initiate objects
*/
bandit_msgs::Joint * desiredJointPos;
ros::Publisher * joint_publisher;
ros::Subscriber * joint_subscriber;
ros::ServiceServer * joint_calib_srv;
ros::AsyncSpinner * spinner;

std::vector<double> * joint_positions;

double deg_to_rad(double deg)
{
	return deg * M_PI / 180;
}

double rad_to_deg(double rad)
{
	return rad * 180 / M_PI;
}

void bandit_master_reset()
{
	for (int k = 0; k < 16; k++){
	
		desiredJointPos->id = k;
		desiredJointPos->angle = deg_to_rad( 0 );
		ros::Rate(5).sleep();
	}
	ROS_INFO("MASTER RESET COMMAND EXECUTED");
}

void jointStatesCallBack(const sensor_msgs::JointStateConstPtr &js)
{
	*joint_positions = js->position;//pulls out all the positions for all joints and stores it to joint_positions
}

void calibrateJoint(const int & p_id, double & minAngle, double & maxAngle, const double & p_angle_increment_deg, double & position_zero)
{
	ROS_INFO("Running calibration on joint: [%i]", p_id);
	double angle_deg = 0; //starting angle
	double lastPose = rad_to_deg( joint_positions->at(p_id) );
	double currentPose = rad_to_deg( joint_positions->at(p_id) );
	bool continueCalibration = true;
	int direction[] = { 0, 0, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, 1, 1};
	position_zero = 0.0;
	minAngle = 0.0;
	maxAngle = 0.0;
	
	position_zero = rad_to_deg( joint_positions->at(p_id) );//get the encoder position at 0 degrees
	ROS_INFO("Position Zero [%f]:", position_zero);
	ROS_INFO("currentPose: %f lastPose: %f", currentPose, lastPose);
	ROS_INFO("desired angle: %f ID: %d", angle_deg, p_id);

	bandit_master_reset();
		
	desiredJointPos->id = p_id;
	bool checker_sum;
	
	while(continueCalibration)
	{	
		angle_deg += direction[p_id] * p_angle_increment_deg;
		desiredJointPos->angle = deg_to_rad(angle_deg);//publishes new angle
		ros::Duration(3).sleep();
		
		currentPose = rad_to_deg( joint_positions->at(p_id) );//stores the value at the p_id-th position in array joint_position to currentPose

		ROS_INFO("currentPose: %f lastPose: %f", currentPose, lastPose);
		ROS_INFO("desired angle: %f ID: %d", angle_deg, p_id);

		if (direction[p_id] == -1){
			checker_sum = currentPose >= lastPose? true:false;
		}
		else
			checker_sum = currentPose <= lastPose? true:false;
		
		if( checker_sum )
		{
			minAngle = direction[p_id] == -1 ? lastPose : minAngle;
			maxAngle = direction[p_id] == 1 ? lastPose : maxAngle;
			
			angle_deg = 0;
			desiredJointPos->angle = deg_to_rad( angle_deg );//reset joint p_id to zero
			ros::Rate(2).sleep();

			ROS_INFO("currentPose: %f lastPose: %f", currentPose, lastPose);
			ROS_INFO("desired angle: %f ID: %d", angle_deg, p_id);
			
			continueCalibration = false;
			
			//continueCalibration = (direction[p_id] == -1);
			//direction = direction == -1 ? 1 : 0;
		}
		else
			lastPose = currentPose;
	}
	bandit_master_reset();//resets all joints to 0
}

void publish_joint_ind()
{
	joint_publisher->publish(*desiredJointPos);
}

/* //this is a modified service call for all joints with srv CalibrateJointAll.srv
bool calibrateJointCallback(bandit_msgs::CalibrateJointAll::Request & req, bandit_msgs::CalibrateJointAll::Response & resp)
{
	double minAngle, maxAngle, position_zero;
	int a = req.Engage;
	for (int joint_i = 0; joint_i < 16; joint_i++){

			calibrateJoint(joint_i, minAngle, maxAngle, 0, 40, position_zero);
			minAngle = minAngle > position_zero ? position_zero - minAngle : minAngle - position_zero;
			maxAngle = maxAngle > position_zero ? maxAngle - position_zero : maxAngle - position_zero;
			resp.MinAngle[joint_i] = minAngle;
			resp.MaxAngle[joint_i] = maxAngle;
	}
	return true;
}
*/

bool calibrateJointCallback(bandit_msgs::CalibrateJoint::Request & req, bandit_msgs::CalibrateJoint::Response & resp)
{
	double minAngle, maxAngle, position_zero;
	calibrateJoint(req.Id, minAngle, maxAngle, req.Angle_Increment, position_zero);	
	minAngle = minAngle > position_zero ? position_zero - minAngle : minAngle - position_zero;
	maxAngle = maxAngle > position_zero ? maxAngle - position_zero : maxAngle - position_zero;
	resp.MinAngle = minAngle;
	resp.MaxAngle = maxAngle;
	return true;
}

int main( int argc, char* argv[] )
{
	ros::init(argc,argv,"bandit_calib");
	
	ros::NodeHandle n;
	joint_publisher = new ros::Publisher;
	*joint_publisher = n.advertise<bandit_msgs::Joint>("joint_ind",10); //creates publisher and publishes to topic "joint_ind"
	
	joint_subscriber = new ros::Subscriber;
	*joint_subscriber = n.subscribe("joint_states", 10, jointStatesCallBack); //creates subscriber and subscribes to topic "joint_states"
	
	joint_calib_srv = new ros::ServiceServer;
	*joint_calib_srv = n.advertiseService("calibrate_joint", calibrateJointCallback);//creates service and can be called by service calibrate_joint
	
	ofstream write_bandit_file;
	
	spinner = new ros::AsyncSpinner(2);//Use 2 threads
	ros::Rate(1).sleep();
	
	spinner->start();//starts threads
	
	desiredJointPos = new bandit_msgs::Joint;
	
	joint_positions = new std::vector<double>;
	
	while(ros::ok())
	{
		publish_joint_ind(); //send out a message to update the joint position
		ros::Rate(20).sleep();
	}
    
    return 0;
  }
