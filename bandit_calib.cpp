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
#include <yaml-cpp/yaml.h>

using namespace std;

/*
	initiate objects
*/
bandit_msgs::Joint * desiredJointPos;
ros::Publisher * joint_publisher;
ros::Subscriber * joint_subscriber;
//ros::ServiceServer * joint_calib_srv;
ros::AsyncSpinner * spinner;

std::vector<double> * joint_positions;

double minAngle = 0;
double maxAngle = 0;
double position_zero = 0;
double offset = 0;
double truezero = 0;

double deg_to_rad(double deg)
{
	return deg * M_PI / 180;
}

double rad_to_deg(double rad)
{
	return rad * 180 / M_PI;
}

void publish_joint_ind()
{
	joint_publisher->publish(*desiredJointPos);
}

void bandit_reset_shoulder()
{
	desiredJointPos->id = 3;
	desiredJointPos->angle = deg_to_rad( 10 );
	publish_joint_ind();
	ros::Rate(1).sleep();

	desiredJointPos->id = 10;
	desiredJointPos->angle = deg_to_rad( 10 );
	publish_joint_ind();
	ros::Rate(1).sleep();
	
	ROS_INFO("SHOULDER RESET COMMAND EXECUTED");
}

void bandit_reset_elbow()
{
	desiredJointPos->id = 5;
	desiredJointPos->angle = deg_to_rad( 90 );
	publish_joint_ind();
	ros::Rate(1).sleep();

	desiredJointPos->id = 12;
	desiredJointPos->angle = deg_to_rad( 90 );
	publish_joint_ind();
	ros::Rate(1).sleep();

	ROS_INFO("ELBOW RESET COMMAND EXECUTED");
}

void bandit_master_reset()
{
	for (int k = 0; k < 16; k++){
	
		desiredJointPos->id = k;
		desiredJointPos->angle = deg_to_rad( 0 );
		publish_joint_ind();
		ros::Rate(5).sleep();
	}
	
	ROS_INFO("MASTER RESET COMMAND EXECUTED");
}

void jointStatesCallBack(const sensor_msgs::JointStateConstPtr &js)
{
	*joint_positions = js->position;//pulls out all the positions for all joints and stores it to joint_positions
}

void calibrateJoint(const int & p_id, const double & p_angle_increment_deg, double & position_zero)
{
	ROS_INFO("Running calibration on joint: [%i]", p_id);
	double angle_deg = 0; //starting angle
	double lastPose = rad_to_deg( joint_positions->at(p_id) );
	double currentPose = rad_to_deg( joint_positions->at(p_id) );
	bool continueCalibration = true;
	int direction[] = { 0, 0, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, 1, 1};
	
	position_zero = rad_to_deg( joint_positions->at(p_id) );//get the encoder position at 0 degrees
	ROS_INFO("Position Zero [%f]:", position_zero);
	ROS_INFO("currentPose: %f lastPose: %f", currentPose, lastPose);
	ROS_INFO("desired angle: %f ID: %d", angle_deg, p_id);

	if (p_id == 2 || p_id == 9){
	
		bandit_reset_shoulder();
	}
	else if (p_id == 3 || p_id == 6 || p_id == 7 || p_id == 8 || p_id == 10 || p_id == 13 || p_id == 14 || p_id == 15){

		bandit_reset_elbow();
	}
	else
		bandit_master_reset();
		
	desiredJointPos->id = p_id;
	bool checker_sum;
	
	while(continueCalibration && ros::ok())
	{	
		angle_deg += direction[p_id] * p_angle_increment_deg;
		desiredJointPos->angle = deg_to_rad(angle_deg);//publishes new angle
		publish_joint_ind();
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
			if (direction[p_id] == -1){
				minAngle = lastPose > position_zero ? position_zero - lastPose : lastPose - position_zero;
				maxAngle = 0.0;
			}
			else {
				maxAngle = lastPose > position_zero ? lastPose - position_zero : lastPose - position_zero;
				minAngle = 0.0;
			}
			//minAngle = direction[p_id] == -1 ? lastPose : minAngle;
			//maxAngle = direction[p_id] == 1 ? lastPose : maxAngle;

			angle_deg = 0;
			desiredJointPos->angle = deg_to_rad( angle_deg );//reset joint p_id to zero
			publish_joint_ind();
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

/* //the service call for one specified joint
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
*/

void true_zero(const int & id)
{	
	switch(id){
		case 0:
			offset = 0;
			truezero = minAngle + offset;
			break;
		case 1:
			ROS_INFO("No Joint Calibration Available");//this joint has no max/min
			break;
		case 2:
			offset = -97;
			truezero = minAngle - offset;
			break;
		case 3:
			offset = (156 + 3); //the 3 is the offset from 0 degrees from starting position
			truezero = maxAngle - offset;
			break;
		case 4:
			offset = -114;
			truezero = minAngle - offset;
			break;
		case 5:
			offset = 119;
			truezero = maxAngle - offset;
			break;
		case 6:
			offset = 87;
			truezero = maxAngle - offset;
			break;
		case 7:
			offset = 28;
			truezero = maxAngle - offset;
			break;
		case 8:
			ROS_INFO("No Joint Calibration Available");//the gripper does not actuate
			break;
		case 9:
			offset = -107;
			truezero = minAngle - offset;
			break;
		case 10:
			offset = (159 + 3);
			truezero = maxAngle - offset;
			break;
		case 11:
			offset = -135;
			truezero = minAngle - offset;
			break;
		case 12:
			offset = 115;
			truezero = maxAngle - offset;
			break;
		case 13:
			offset = 98;
			truezero = maxAngle - offset;
			break;
		case 14:
			offset = 28;
			truezero = maxAngle - offset;
			break;
		case 15:
			ROS_INFO("No Joint Calibration Available");//the gripper does not actuate
			break;
		default:
			ROS_INFO("No Joint Calibration Available");
			break;
	}
	
}

int main( int argc, char* argv[] )
{
	ros::init(argc,argv,"bandit_calib");
	
	ros::NodeHandle n;
	joint_publisher = new ros::Publisher;
	*joint_publisher = n.advertise<bandit_msgs::Joint>("joint_ind",10); //creates publisher and publishes to topic "joint_ind"
	
	joint_subscriber = new ros::Subscriber;
	*joint_subscriber = n.subscribe("joint_states", 10, jointStatesCallBack); //creates subscriber and subscribes to topic "joint_states"
	
	ofstream write_bandit_file;//declares output stream
	write_bandit_file.open("Bandit_Calibration_File.txt");
	YAML::Emitter bandit_file;
	
	//joint_calib_srv = new ros::ServiceServer;
	//*joint_calib_srv = n.advertiseService("calibrate_joint", calibrateJointCallback);//creates service and can be called by service calibrate_joint
	
	spinner = new ros::AsyncSpinner(2);//Use 2 threads
	ros::Rate(1).sleep();
	
	spinner->start();//starts threads
	
	desiredJointPos = new bandit_msgs::Joint;
	
	joint_positions = new std::vector<double>;
	
	if(ros::ok()){
		
		for ( int z_id = 0; z_id < 16; z_id++){
			if(z_id == 6 || z_id == 7 || z_id == 8 || z_id == 13 || z_id == 14 || z_id == 15){
				calibrateJoint(z_id, 10, position_zero);
				true_zero(z_id);
			}
			else if(z_id == 4 || z_id == 11){
				calibrateJoint(z_id, 30, position_zero);
				true_zero(z_id);
			}
			else {
				calibrateJoint(z_id, 20, position_zero);
				true_zero(z_id);				
			}
			
			write_bandit_file << "Joint ID: " << z_id << " True Zero: " << truezero << " Offset: " << offset << "\n";
			write_bandit_file << "MaxAngle: " << maxAngle << " MinAngle: " << minAngle << "\n \n";
			
			bandit_file << YAML::BeginMap;
			bandit_file << YAML::Key << "Joint ID:";
			bandit_file << YAML::Value << z_id;
			bandit_file << YAML::Literal << "\n";
			bandit_file << YAML::Key << "True Zero:";
			bandit_file << YAML::Value << truezero;
			bandit_file << YAML::Literal << "\n";
			bandit_file << YAML::Key << "Offset:";
			bandit_file << YAML::Value << offset;
			bandit_file << YAML::Literal << "\n";
			bandit_file << YAML::Key << "MaxAngle:";
			bandit_file << YAML::Value << maxAngle;
			bandit_file << YAML::Literal << "\n";
			bandit_file << YAML::Key << "MinAngle:";
			bandit_file << YAML::Value << minAngle;
			bandit_file << YAML::Literal << "\n";
			bandit_file << YAML::EndMap;
		}
		ros::spinOnce();
		ros::Rate(20).sleep();
		write_bandit_file.close();
	}
	else
		return 0;
	
    return 0;
  }
