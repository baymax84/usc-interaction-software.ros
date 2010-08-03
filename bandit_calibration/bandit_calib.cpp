#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "bandit_msgs/JointArray.h"
#include "bandit_msgs/Params.h"
#include "math.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
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
int direction[19] = { -1, 0, -1, 1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, 1, -1, 0, 0, 0};

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

void bandit_master_reset()
{
	for (int ii = 0; ii < 16; ii++){
		desiredJointPos->id = ii;
		desiredJointPos->angle = deg_to_rad( 0 );
		publish_joint_ind();
		ros::Rate(3).sleep();
	}
	ROS_INFO("MASTER RESET COMMAND EXECUTED");
}

void jointStatesCallBack(const sensor_msgs::JointStateConstPtr &js)
{
	*joint_positions = js->position;//pulls out all the positions for all joints and stores it to joint_positions
}

//designed to check that the true maxangle is recorded <not yet completed>
bool joint_checker(const int& p_id, double& lastPose, const int& direction, bool& checker_sum, const double& p_angle_increment_deg){
	desiredJointPos->id = p_id;
	double angledeg;
	angledeg += direction * 2 * p_angle_increment_deg;
	desiredJointPos->angle = deg_to_rad(angledeg);//publishes new angle
	publish_joint_ind();
	ros::Duration(2).sleep();
	
	double current_pos = rad_to_deg( joint_positions->at(p_id) );
	if (direction == -1){
		checker_sum = current_pos >= lastPose? true:false;
	}
	else{
		checker_sum = current_pos <= lastPose? true:false;
	}
	return checker_sum;
}

void calibrateJoint(const int & p_id, const double & p_angle_increment_deg, double & position_zero)
{
	ROS_INFO("Running calibration on joint: [%i]", p_id);
	double angle_deg = 0; //starting angle
	double lastPose = rad_to_deg( joint_positions->at(p_id) );
	double currentPose = rad_to_deg( joint_positions->at(p_id) );
	bool continueCalibration = true;
	
	position_zero = rad_to_deg( joint_positions->at(p_id) );//get the encoder position at 0 degrees
	ROS_INFO("Position Zero [%f]:", position_zero);
	ROS_INFO("currentPose: %f lastPose: %f", currentPose, lastPose);
	ROS_INFO("desired angle: %f ID: %d", angle_deg, p_id);
	
	bandit_master_reset();
		
	desiredJointPos->id = p_id;
	bool checker_sum;
	
	while(continueCalibration && ros::ok())
	{	
		angle_deg += direction[p_id] * p_angle_increment_deg;
		desiredJointPos->angle = deg_to_rad(angle_deg);//publishes new angle
		publish_joint_ind();
		ros::Duration(2).sleep();
		
		currentPose = rad_to_deg( joint_positions->at(p_id) );//stores the value at the p_id-th position in array joint_position to currentPose

		ROS_INFO("currentPose: %f lastPose: %f", currentPose, lastPose);
		ROS_INFO("desired angle: %f ID: %d", angle_deg, p_id);

		if (direction[p_id] == -1){
			checker_sum = currentPose >= lastPose? true:false;
		}
		else{
			if (p_id == 2 || p_id == 4 || p_id == 9 || p_id == 11){
				checker_sum = currentPose < lastPose? true:false;
			}
			else
				checker_sum = currentPose <= lastPose? true:false;
		}
		
		if( checker_sum ){
				joint_checker(p_id, lastPose, direction[p_id], checker_sum, p_angle_increment_deg);
		}
		
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
	bandit_master_reset();//resets all joints to fixed position
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

/*
 * This function is used to set the offset for the physical hardware calibration limits for Bandit II
 * First run bandit_calib_version1 and use the service call to determine the min/max angle of the joints and input below
 * The Second value of (XXX + XX) is measure for the zero position of hands down, palm facing inwards, and forearm facing outwards
 */

void true_zero(const int & id)
{	
	switch(id){
		case 0:
			offset = -27; //this joint is inoperational on Bandit 2
			truezero = minAngle - offset;
			break;
		case 1:
			offset = 0;
			truezero = minAngle - offset;
			ROS_INFO("No Joint Calibration Available");//this joint has no max/min
			break;
		case 2:
			offset = (-157 + 60);
			truezero = minAngle - offset;
			if (truezero > 0){
				truezero = - truezero;
			}
			break;
		case 3:
			offset = (85 + 90);
			truezero = maxAngle - offset;
			if (truezero > 0){
				truezero = - truezero;
			}
			break;
		case 4:
			offset = (-85 + 30);
			truezero = minAngle - offset;
			if (truezero > 0){
				truezero = - truezero;
			}
			break;
		case 5:
			offset = (68 + 60);
			truezero = maxAngle - offset;
			if (truezero < 0){
				truezero = - truezero;
			}
			break;
		case 6:
			offset = 77;
			truezero = maxAngle - offset;
			if (truezero > 0){
				truezero = - truezero;
			}
			break;
		case 7:
			offset = 28;
			truezero = maxAngle - offset;
			break;
		case 8:
			offset = 0;
			truezero = 0.6;
			ROS_INFO("No Joint Calibration Available");//this joint is inoperational on Bandit 2
			break;
		case 9:
			offset = (-158 + 60);
			truezero = minAngle - offset;
			if (truezero < 0){
				truezero = - truezero;
			}
			break;
		case 10:
			offset = (80 + 90);
			truezero = maxAngle - offset;
			if (truezero < 0){
				truezero = - truezero;
			}
			break;
		case 11:
			offset = (-150 + 30);
			truezero = minAngle - offset;
			if (truezero < 0){
				truezero = - truezero;
			}
			break;
		case 12:
			offset = (61 + 60);
			truezero = maxAngle - offset;
			if (truezero > 0){
				truezero = - truezero;
			}
			break;
		case 13:
			offset = 87;
			truezero = maxAngle - offset;
			if (truezero < 0){
				truezero = - truezero;
			}
			break;
		case 14:
			offset = 28; //this joint moves in the opposite direction
			truezero = maxAngle - offset;
			break;
		case 15:
			offset = 0;
			truezero = 0.5;
			ROS_INFO("No Joint Calibration Available");//this joint is inoperational on Bandit 2
			break;
		case 16:
			offset = 0;
			truezero = 0.2;
			ROS_INFO("No Joint Calibration Available");
			break;
		case 17:
			offset = 0;
			truezero = 0.25;
			ROS_INFO("No Joint Calibration Available");
			break;
		case 18:
			offset = 0;
			truezero = 0.25;
			ROS_INFO("No Joint Calibration Available");
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
	
	//joint_calib_srv = new ros::ServiceServer;
	//*joint_calib_srv = n.advertiseService("calibrate_joint", calibrateJointCallback);//creates service and can be called by service calibrate_joint
	
	spinner = new ros::AsyncSpinner(2);//Use 2 threads
	ros::Rate(1).sleep();
	
	spinner->start();//starts threads
	
	desiredJointPos = new bandit_msgs::Joint;
	
	joint_positions = new std::vector<double>;
	
	std::string filename = "Bandit_Calibration_File.yaml";
	ofstream write_bandit_file;//declares output stream
	write_bandit_file.open(filename.c_str());//creates file Bandit_Calibration_File.yaml
	YAML::Emitter emitter;//declares output emitter for yaml formatting
	emitter << YAML::BeginSeq;
	
	if(ros::ok()){
		
		for ( int z_id = 0; z_id < 19; z_id++){
			if(z_id == 3 || z_id == 4 || z_id == 10 || z_id == 11){
				calibrateJoint(z_id, 30, position_zero);
				true_zero(z_id);
			}
			else if(z_id == 2 || z_id == 9){
				calibrateJoint(z_id, 40, position_zero);
				true_zero(z_id);
			}
			else {
				calibrateJoint(z_id, 10, position_zero);
				true_zero(z_id);				
			}
			
			emitter << YAML::BeginMap;
			emitter << YAML::Key << "Joint ID" << YAML::Value << z_id;
			emitter << YAML::Key << "Direction" << YAML::Value << direction[z_id];
			emitter << YAML::Key << "True Zero" << YAML::Value << truezero;
			emitter << YAML::Key << "Offset" << YAML::Value << offset;
			emitter << YAML::Key << "Max Angle" << YAML::Value << maxAngle;
			emitter << YAML::Key << "Min Angle" << YAML::Value << minAngle;
			emitter << YAML::EndMap;
			std::cout << emitter.c_str() << endl;	
		}
		ros::spinOnce();
		ros::Rate(20).sleep();
		emitter << YAML::EndSeq;
		write_bandit_file << emitter.c_str();
		write_bandit_file.close();
	}
	else
		return 0;
	
    return 0;
  }
