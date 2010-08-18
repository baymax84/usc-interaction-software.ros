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

ros::Publisher joint_publisher;
sensor_msgs::JointState bandit_state;
YAML::Emitter emitter;

double offset = 0;
double truezero = 0;
int direction[19] = { -1, 0, -1, 1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, 1, -1, 0, 0, 0}; //direction of testing
int direction_Emit[19] = {-1,-1,1,1,1,-1,1,1,1,-1,-1,-1,1,-1,-1,1,1,1,-1};//direction of movement needed in bandit_node for bandit to move correctly

#define RTOD(a) ((a) * 180.0/ M_PI )
#define DTOR(a) ((a) * M_PI/ 180.0 )


// Takes the encoder information about joint positiosn from the Bandit II encoders

void jointStatesCallBack(const sensor_msgs::JointStateConstPtr &js)
{
	bandit_state = *js;
}

//Resets all joints to its zero position

void bandit_master_reset()
{
  bandit_msgs::JointArray joints;
  joints.joints.resize(16);

	for (int ii = 0; ii < 16; ii++){
		joints.joints[ii].id = ii;
		joints.joints[ii].angle = DTOR( 0 );
    joint_publisher.publish(joints);
	}
	ROS_INFO("MASTER RESET COMMAND EXECUTED");
  ros::Duration(1).sleep();
}

void true_zero(const int & id, double minAngle, double maxAngle)
{	
  double offset = 0, truezero = 0;

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

	emitter << YAML::Key << "TrueZero" << YAML::Value << truezero;
	emitter << YAML::Key << "Offset" << YAML::Value << offset;
	
}


//calibrates a single joint
void calibrate_joint( unsigned int j_id )
{
	ROS_INFO("Running calibration on joint: [%i]", j_id);

  // set high pos
  bandit_msgs::JointArray joints;
  joints.joints.resize(1);
  joints.joints[0].id = j_id;
  joints.joints[0].angle = direction[j_id] * DTOR(179.0);
  joint_publisher.publish(joints);

  // wait for a bit
  ros::Duration(3.0).sleep();

  // check state
  double max_angle = direction[j_id] * RTOD(bandit_state.position[j_id]);

  // set low pos
  joints.joints[0].id = j_id;
  joints.joints[0].angle = direction[j_id]* DTOR(-179.0);
  joint_publisher.publish(joints);

  // wait for a bit
  ros::Duration(4.0).sleep();

  // check state
  double min_angle = direction[j_id]*RTOD(bandit_state.position[j_id]);

  ROS_INFO( "[min max]: [%0.2f %0.2f]", min_angle, max_angle );

  joints.joints[0].angle = DTOR(0);
  joint_publisher.publish(joints);

  ros::Duration(2.0).sleep();
  
  //output values for the YAML File
	emitter << YAML::BeginMap;
	emitter << YAML::Key << "JointID" << YAML::Value << (int) j_id;
	emitter << YAML::Key << "Direction" << YAML::Value << direction_Emit[j_id];
	emitter << YAML::Key << "MaxAngle" << YAML::Value << max_angle;
	emitter << YAML::Key << "MinAngle" << YAML::Value << min_angle;

  true_zero( j_id, min_angle, max_angle );

	emitter << YAML::EndMap;
	std::cout << emitter.c_str() << endl;	
}


/*
	// Checks the MaxAngle/MinAngle recorded is the true hardware limit by publishing more and checking

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


	// The Main function that calibrates the joints by publishing increment angles and checking

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
		
		// Checks if the Joint Angle has not changed or has gone in the reverse direction
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
		
		//Checks again
		if( checker_sum ){
				joint_checker(p_id, lastPose, direction[p_id], checker_sum, p_angle_increment_deg);
		}
		
		//Normalizes the encoder information
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
*/
/*
 * This function is used to set the offset for the physical hardware calibration limits for Bandit II
 * First run bandit_calib_version1 and use the service call to determine the min/max angle of the joints and input below
 * The Second value of (XXX + XX) is measure for the zero position of hands down, palm facing inwards, and forearm facing outwards
 */

int main( int argc, char* argv[] )
{
	ros::init(argc,argv,"bandit_calib");
	
	ros::NodeHandle n;
  ros::NodeHandle nh_priv("~");
	joint_publisher = n.advertise<bandit_msgs::JointArray>("joint_cmd",10);
	ros::Subscriber joint_subscriber = n.subscribe("joint_states", 10, jointStatesCallBack);

  ros::AsyncSpinner spinner(2);//Use 2 threads
  spinner.start();//starts threads
  ros::Duration(1).sleep();
	

  // yaml file stuff

	std::string filename;
  nh_priv.param( "filename", filename, std::string("Bandit_Calibration_File.yaml"));
	ofstream write_bandit_file;//declares output stream
	write_bandit_file.open(filename.c_str());//creates file Bandit_Calibration_File.yaml
	emitter << YAML::BeginSeq;
	

  if(ros::ok())
  {
    for( unsigned int i = 0; i < 19; i++ )
    {
      if( direction[i] != 0 )
        calibrate_joint(i);
    }
	}
	else
		return 0;

    // close up yaml file
		emitter << YAML::EndSeq;
		write_bandit_file << emitter.c_str();
		write_bandit_file.close();

    return 0;
  }
