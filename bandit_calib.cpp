#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "bandit_msgs/JointArray.h"
#include "bandit_msgs/Params.h"
#include "bandit_msgs/CalibrateJointAll.h"
#include "math.h"
#include <vector>

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

void jointStatesCallBack(const sensor_msgs::JointStateConstPtr &js)
{
	*joint_positions = js->position;//pulls out all the positions for all joints and stores it to joint_positions
	//ROS_INFO("Updated joint positions");
}

void calibrateJoint(const int & p_id, double & minAngle, double & maxAngle, const double & p_angle_deg, const double & p_angle_increment_deg)
{
	ROS_INFO("Running calibration on joint: [%i]", p_id);
	double angle_deg = 0;// -p_angle_increment_deg;//deg_to_rad(p_angle_deg); //so this is the starting angle?
	double lastPose = rad_to_deg( joint_positions->at(p_id) );
	double currentPose = rad_to_deg( joint_positions->at(p_id) );
	int direction = -1;
	bool continueCalibration = true;
	minAngle = 0.0;
	maxAngle = 0.0;
	
	desiredJointPos->id = p_id;
	
	desiredJointPos->angle = deg_to_rad( 0 );
	ros::Rate(2).sleep();
	//lastPose = rad_to_deg( joint_positions->at(p_id) );
	
	int counter = 0;
	double checker_array[3];
	while(continueCalibration)
	{	

		angle_deg += direction * p_angle_increment_deg;
		desiredJointPos->angle = deg_to_rad(angle_deg);
		ros::Rate(1).sleep();
		
		currentPose = rad_to_deg( joint_positions->at(p_id) );//stores the value at the p_id-th position in array joint_position to currentPose
		checker_array[counter % 3] = fabs(currentPose - lastPose);

		ROS_INFO("currentPose: %f lastPose: %f", currentPose, lastPose);
		ROS_INFO("desired angle: %f ID: %d", angle_deg, p_id);
		ROS_INFO("change in pose: %f:", fabs(currentPose - lastPose));

		if( (checker_array[0] < 1) && (checker_array[1] < 1) && (checker_array[2] < 1) )
		{
			minAngle = direction == -1 ? rad_to_deg( joint_positions->at(p_id) ) : minAngle;
			maxAngle = direction == 1 ? rad_to_deg( joint_positions->at(p_id) ) : maxAngle;
			
			//reset joint to 0
			angle_deg = 0;
			desiredJointPos->angle = deg_to_rad(angle_deg);
			ros::Rate(2).sleep();

			lastPose = rad_to_deg( joint_positions->at(p_id) );//lastPose = minAngle
			
			ROS_INFO("currentPose: %f lastPose: %f", currentPose, lastPose);
			ROS_INFO("desired angle: %f ID: %d", angle_deg, p_id);
			ROS_INFO("change in pose: %f:", fabs(currentPose - lastPose));

			continueCalibration = (direction == -1);
			direction = direction == -1 ? 1 : 0;
		}
		else
			lastPose = currentPose;

		counter++;
	}

	//ROS_INFO("Joint ID [%i] Maximum (+) [%f] Maximum (-): [%f]", p_id, minAngle, maxAngle);
}

void publish_joint_ind()
{
	//ROS_INFO("publshing desired joint info");
	joint_publisher->publish(*desiredJointPos);
}

bool calibrateJointCallback(bandit_msgs::CalibrateJointAll::Request & req, bandit_msgs::CalibrateJointAll::Response & resp)
{
	double minAngle, maxAngle;
	int a = req.Engage;
	for (int joint_i = 0; joint_i < 14; joint_i++){
		
		if (joint_i !=7 || joint_i !=8){
			calibrateJoint(joint_i, minAngle, maxAngle, 0, 10);
			resp.MinAngle = minAngle;
			resp.MaxAngle = maxAngle;
		}
	}
	return true;
}

/*
bool calibrateJointCallback(bandit_msgs::CalibrateJoint::Request & req, bandit_msgs::CalibrateJoint::Response & resp)
{
	double minAngle, maxAngle;
	calibrateJoint(req.Id, minAngle, maxAngle, 0.0, req.Angle_Increment);
	resp.MinAngle = minAngle;
	resp.MaxAngle = maxAngle;
	return true;
}
*/

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
	
	spinner = new ros::AsyncSpinner(2);//Use 2 threads
	
	ros::Rate(1).sleep();
	
	spinner->start();//starts threads
	
	desiredJointPos = new bandit_msgs::Joint;
	
	joint_positions = new std::vector<double>;
	
	while(ros::ok())
	{
		publish_joint_ind(); //send out a message to update the joint position
		//ros::spinOnce();
		ros::Rate(20).sleep();
	}
    
    return 0;
  }
