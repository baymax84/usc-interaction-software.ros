#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "bandit_msgs/JointArray.h"
#include "bandit_msgs/Params.h"
#include "bandit_msgs/CalibrateJoint.h"
#include "math.h"
#include <vector>

bandit_msgs::Joint * desiredJointPos;
ros::Publisher * joint_publisher;
ros::Subscriber * joint_subscriber;
ros::ServiceServer * joint_calib_srv;

std::vector<double> * joint_positions;

double deg_to_rad(double deg)
{
	return deg * M_PI / 180;
}

double rad_to_deg(double rad)
{
	return rad * 180 / M_PI;
}

void jointStatesCallBack(const sensor_msgs::JointStateConstPtr &js)
{
	*joint_positions = js->position;
	ROS_INFO("Updated joint positions");
}

void calibrateJoint(const int & p_id, double & minAngle, double & maxAngle, const double & p_angle_deg, const double & p_angle_increment_deg = 20)
{
	ROS_INFO("Running calibration on joint: [%i]", p_id);
	double angle_deg = p_angle_deg;//deg_to_rad(p_angle_deg); //so this is the starting angle?
	double lastPose = joint_positions->at(p_id);
	double currentPose;
	int direction = -1; //
	bool continueCalibration = true;
	minAngle = 0.0;
	maxAngle = 0.0;
	
	desiredJointPos->id = p_id;
	
	while(continueCalibration)
	{
		ros::Rate(2).sleep();
		
		angle_deg += direction * p_angle_increment_deg;
		
		currentPose = rad_to_deg( joint_positions->at(p_id) );
		lastPose = currentPose;
		
		desiredJointPos->angle = deg_to_rad(angle_deg);
		
		if(abs(currentPose - lastPose) < 1.0)
		{
			minAngle = direction == -1 ? angle_deg : minAngle;
			maxAngle = direction == 1 ? angle_deg : maxAngle;
			
			continueCalibration = (direction == -1);
			direction = direction == -1 ? 1 : 0;
		}
	}
	ROS_INFO("Joint ID [%i] Maximum (+) [%f] Maximum (-): [%f]", p_id, minAngle, maxAngle);
}

void publish_joint_ind()
{
	joint_publisher->publish(*desiredJointPos);
}

bool calibrateJointCallback(bandit_msgs::CalibrateJoint::Request & req, bandit_msgs::CalibrateJoint::Response & resp)
{
	double minAngle, maxAngle;
	calibrateJoint(req.Id, minAngle, maxAngle, 0.0, 20.0);
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
	*joint_calib_srv = n.advertiseService("calibrate_joint", calibrateJointCallback);
	//ros::AsyncSpinner spinner(2);//Use 2 threads
	
	desiredJointPos = new bandit_msgs::Joint;
	
	joint_positions = new std::vector<double>;
	
	while(ros::ok())
	{
		ros::spinOnce();
		publish_joint_ind(); //send out a message to update the joint position
		ros::Rate(10).sleep();
	}
    
    return 0;
  }
