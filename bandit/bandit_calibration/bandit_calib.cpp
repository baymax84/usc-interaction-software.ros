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
ros::AsyncSpinner * spinner;

std::vector<double> * joint_positions;

double minAngle = 0;
double maxAngle = 0;
double position_zero = 0;
double offset = 0;
double truezero = 0;
int direction[19] = { -1, 0, -1, 1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, 1, -1, 0, 0, 0}; //direction of testing
int direction_Emit[19] = {-1,-1,1,1,1,-1,1,1,1,-1,-1,-1,1,-1,-1,1,1,1,-1};//direction of movement needed in bandit_node for bandit to move correctly

/*
	Converts degrees to radians and outputs radians
*/
double deg_to_rad(double deg)
{
	return deg * M_PI / 180;
}

/*
	Converts radiangs to degrees and outputs degrees
*/
double rad_to_deg(double rad)
{
	return rad * 180 / M_PI;
}

/*
	Publishes desiredJointPos (a vector of joint id and joint angle)
*/
void publish_joint_ind()
{
  bandit_msgs::JointArray ja;
  ja.joints.push_back(*desiredJointPos);
	joint_publisher->publish(ja);
}

/*
	Resets all joints to its zero position
*/
void bandit_master_reset()
{
  for (int ii = 0; ii < 16; ii++){
		desiredJointPos->id = ii;
		desiredJointPos->angle = deg_to_rad( 0 );
    }
  publish_joint_ind();

	for (int ii = 0; ii < 16; ii++){
		desiredJointPos->id = ii;
		desiredJointPos->angle = deg_to_rad( 0 );
		//publish_joint_ind();
		ros::Rate(3).sleep();
	}
	//publish_joint_ind();
	//ros::Rate(3).sleep();
	ROS_INFO("MASTER RESET COMMAND EXECUTED");
}

/*
	Takes the encoder information about joint positiosn from the Bandit II encoders
*/
void jointStatesCallBack(const sensor_msgs::JointStateConstPtr &js)
{
	*joint_positions = js->position;//pulls out all the positions for all joints and stores it to joint_positions
}

/*
	Checks the MaxAngle/MinAngle recorded is the true hardware limit by publishing more and checking
*/
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

/*
	The Main function that calibrates the joints by publishing increment angles and checking
*/
void maxAdjust(double maxvalue, const int & p_id, double & currentmax){
  double angle_deg = maxvalue - 20.0;
  double currentPose = maxvalue;
  double lastPose = maxvalue;
  currentmax = maxvalue;
  double diff_thresh = 10.0, pose_diff = 0.0;

  desiredJointPos->id = p_id;
  desiredJointPos->angle = deg_to_rad(angle_deg);
  publish_joint_ind();

  ROS_INFO("Adjusting max on joint: [%i]....", p_id);

  ros::Duration(2).sleep();
  ros::spinOnce();

  // Go 5 degrees past max
  angle_deg = maxvalue + 5.0;
  desiredJointPos->angle = deg_to_rad(angle_deg);
  publish_joint_ind();
  ROS_INFO("Trying max + 5.0...");
  ros::Duration(2).sleep();
  ros::spinOnce();

  currentPose = rad_to_deg( joint_positions->at(p_id) );

  pose_diff = fabs(currentPose - lastPose);

  // Adjust max value
  if(currentPose > currentmax)
    currentmax = currentPose;

  // Joint turned around
  if(currentPose < lastPose && pose_diff > diff_thresh){
    ROS_INFO("joint turned...");

    // Reset to base position
    angle_deg = maxvalue - 20.0;
    desiredJointPos->angle = deg_to_rad(angle_deg);
    publish_joint_ind();
    ros::Duration(2).sleep();
    ros::spinOnce();

    // Go in between
    angle_deg = maxvalue + 2.0;
    desiredJointPos->angle = deg_to_rad(angle_deg);
    publish_joint_ind();
    ROS_INFO("Trying max + 2.0...");
    ros::Duration(2).sleep();
    ros::spinOnce();

    // Get current position
    currentPose = rad_to_deg( joint_positions->at(p_id) );
    pose_diff = fabs(currentPose - lastPose);

    // Adjust max value
    if(currentPose > currentmax)
      currentmax = currentPose;
  }
  // Keep going
  else{
    lastPose = currentPose;

    // Reset to base position
    angle_deg = maxvalue - 20.0;
    desiredJointPos->angle = deg_to_rad(angle_deg);
    publish_joint_ind();
    ros::Duration(2).sleep();
    ros::spinOnce();

    // Go further
    angle_deg = maxvalue + 10.0;
    desiredJointPos->angle = deg_to_rad(angle_deg);
    publish_joint_ind();
    ROS_INFO("Trying max + 10.0...");
    ros::Duration(2).sleep();
    ros::spinOnce();

    // Get current position
    currentPose = rad_to_deg( joint_positions->at(p_id) );
    pose_diff = fabs(currentPose - lastPose);

    // Adjust max value
    if(currentPose > currentmax)
      currentmax = currentPose;

    // Joint turned around
    if(currentPose < lastPose && pose_diff > diff_thresh){
      ROS_INFO("joint turned...");

      // Reset to base position
      angle_deg = maxvalue - 20.0;
      desiredJointPos->angle = deg_to_rad(angle_deg);
      publish_joint_ind();
      ros::Duration(2).sleep();
      ros::spinOnce();

      // Go in between
      angle_deg = maxvalue + 7.0;
      desiredJointPos->angle = deg_to_rad(angle_deg);
      publish_joint_ind();
      ROS_INFO("Trying max + 7.0...");
      ros::Duration(2).sleep();
      ros::spinOnce();
    
      // Get current position
      currentPose = rad_to_deg( joint_positions->at(p_id) );
      pose_diff = fabs(currentPose - lastPose);

      // Adjust max value
      if(currentPose > currentmax)
	currentmax = currentPose;
    }
  }
  // Reset to base position
  angle_deg = maxvalue - 20.0;
  desiredJointPos->angle = deg_to_rad(angle_deg);
  publish_joint_ind();
  ros::Duration(2).sleep();
  ros::spinOnce();
  // Reset to base position
  angle_deg = 0.0;
  desiredJointPos->angle = deg_to_rad(angle_deg);
  publish_joint_ind();
  ros::Duration(2).sleep();
  ros::spinOnce();
}

void minAdjust(double minvalue, const int & p_id, double & currentmin){
  double angle_deg = minvalue + 20.0;
  double currentPose = minvalue;
  double lastPose = minvalue;
  currentmin = minvalue;
  double diff_thresh = 10.0, pose_diff = 0.0;

  desiredJointPos->id = p_id;
  desiredJointPos->angle = deg_to_rad(angle_deg);
  publish_joint_ind();

  ROS_INFO("Adjusting min on joint: [%i]....", p_id);

  ros::Duration(2).sleep();
  ros::spinOnce();

  // Go 5 degrees past min
  angle_deg = minvalue - 5.0;
  desiredJointPos->angle = deg_to_rad(angle_deg);
  publish_joint_ind();
  ROS_INFO("Trying min - 5.0...");
  ros::Duration(2).sleep();

  currentPose = rad_to_deg( joint_positions->at(p_id) );

  pose_diff = fabs(currentPose - lastPose);

  // Adjust min value
  if(currentPose < currentmin)
    currentmin = currentPose;

  // Joint turned around
  if(currentPose > lastPose && pose_diff > diff_thresh){
    ROS_INFO("joint turned...");

    // Reset to base position
    angle_deg = minvalue + 20.0;
    desiredJointPos->angle = deg_to_rad(angle_deg);
    publish_joint_ind();
    ros::Duration(2).sleep();
    ros::spinOnce();

    // Go in between
    angle_deg = minvalue - 2.0;
    desiredJointPos->angle = deg_to_rad(angle_deg);
    publish_joint_ind();
    ROS_INFO("Trying min - 2.0...");
    ros::Duration(2).sleep();

    // Get current position
    currentPose = rad_to_deg( joint_positions->at(p_id) );
    pose_diff = fabs(currentPose - lastPose);

    // Adjust min value
    if(currentPose < currentmin)
      currentmin = currentPose;
  }
  // Keep going
  else{
    lastPose = currentPose;

    // Reset to base position
    angle_deg = minvalue + 20.0;
    desiredJointPos->angle = deg_to_rad(angle_deg);
    publish_joint_ind();
    ros::Duration(2).sleep();
    ros::spinOnce();

    // Go further
    angle_deg = minvalue - 10.0;
    desiredJointPos->angle = deg_to_rad(angle_deg);
    publish_joint_ind();
    ROS_INFO("Trying min - 10.0...");
    ros::Duration(2).sleep();
    ros::spinOnce();

    // Get current position
    currentPose = rad_to_deg( joint_positions->at(p_id) );
    pose_diff = fabs(currentPose - lastPose);

    // Adjust min value
    if(currentPose < currentmin)
      currentmin = currentPose;

    // Joint turned around
    if(currentPose > lastPose && pose_diff > diff_thresh){
      ROS_INFO("joint turned...");

      // Reset to base position
      angle_deg = minvalue + 20.0;
      desiredJointPos->angle = deg_to_rad(angle_deg);
      publish_joint_ind();
      ros::Duration(2).sleep();
      ros::spinOnce();

      // Go in between
      angle_deg = minvalue - 7.0;
      desiredJointPos->angle = deg_to_rad(angle_deg);
      publish_joint_ind();
      ROS_INFO("Trying min - 7.0...");
      ros::Duration(2).sleep();
      ros::spinOnce();
    
      // Get current position
      currentPose = rad_to_deg( joint_positions->at(p_id) );
      pose_diff = fabs(currentPose - lastPose);

      // Adjust min value
      if(currentPose < currentmin)
	currentmin = currentPose;
    }
  }

  // Reset to base position
  angle_deg = minvalue + 20.0;
  desiredJointPos->angle = deg_to_rad(angle_deg);
  publish_joint_ind();
  ros::Duration(2).sleep();
  ros::spinOnce();
  // Reset to base position
  angle_deg = 0.0;
  desiredJointPos->angle = deg_to_rad(angle_deg);
  publish_joint_ind();
  ros::Duration(2).sleep();
  ros::spinOnce();
}

void calibrateJoint(const int & p_id, const double & p_angle_increment_deg, 
		    double & position_zero, double & final_max, double & final_min)
{
	ROS_INFO("Running calibration on joint: [%i]", p_id);
	double angle_deg = 0; //starting angle
	double lastPose = rad_to_deg( joint_positions->at(p_id) );
	double currentPose = rad_to_deg( joint_positions->at(p_id) );
	bool continueCalibration = true;
	
	//bandit_master_reset();

	//ros::Duration(2).sleep();

	lastPose = rad_to_deg( joint_positions->at(p_id) );
	currentPose = rad_to_deg( joint_positions->at(p_id) );

	position_zero = rad_to_deg( joint_positions->at(p_id) );//get the encoder position at 0 degrees
	ROS_INFO("Position Zero [%f]:", position_zero);
	ROS_INFO("   currentPose: %f lastPose: %f", currentPose, lastPose);
	ROS_INFO("   desired angle: %f ID: %d", angle_deg, p_id);
		
	desiredJointPos->id = p_id;
	bool checker_sum = false;
	double diff_thresh = 3.0;
	double maxvalue = 0.0, minvalue = 0.0;
	int counter = 0;
	lastPose = 0.0;

	// Find max
	ROS_INFO("Finding max joint value....\n");
	while(ros::ok()){
	  angle_deg += p_angle_increment_deg;
	  desiredJointPos->angle = deg_to_rad(angle_deg);//publishes new angle
	  publish_joint_ind();
	  ros::Duration(2).sleep();

	  currentPose = rad_to_deg( joint_positions->at(p_id) );
	  if(counter == 0){
	    lastPose = currentPose;
	    maxvalue = currentPose;
	  }

	  ROS_INFO("   currentPose: %f lastPose: %f", currentPose, lastPose);
	  ROS_INFO("   desired angle: %f ID: %d", angle_deg, p_id);

	  double posediff = fabs(currentPose - lastPose);

	  //if(currentPose > lastPose && posediff > diff_thresh)
	  //  continue;
	  if(currentPose < lastPose && posediff > diff_thresh){
	    ROS_INFO("   joint turned!!**");
	    if(lastPose > maxvalue)
	      maxvalue = lastPose;
	    break;
	  }
	  //if(currentPose < lastPose && posediff < diff_thresh){
	  //  maxvalue = lastPose;
	  //  break;
	  //}
	  //if(currentPose > lastPose && posediff < diff_thresh){
	  //  maxvalue = currentPose;
	  //  break;
	  //}
	  if(counter > 3 && (fabs(angle_deg-currentPose) > p_angle_increment_deg*1.5) && (posediff < diff_thresh)){
	    if(currentPose > maxvalue)
	      maxvalue = currentPose;
	    break;
	  }

	  if(currentPose > maxvalue)
	    maxvalue = currentPose;

	  counter++;
	  lastPose = currentPose;
	  ros::spinOnce();
	  
	}
	ROS_INFO("Found max value!: %f ID: %d", maxvalue, p_id);

	maxAdjust(maxvalue,p_id,final_max);

	ROS_INFO("Final adjusted max value: %f ID: %d", final_max, p_id);

	//bandit_master_reset();

	ros::Duration(2).sleep();

	angle_deg = 0.0;
	lastPose = 0.0;//rad_to_deg( joint_positions->at(p_id) );
	currentPose = 0.0;//rad_to_deg( joint_positions->at(p_id) );
	counter = 0;

	// Find min
	ROS_INFO("Finding min joint value....\n");
	while(ros::ok()){
	  angle_deg -= p_angle_increment_deg;
	  desiredJointPos->angle = deg_to_rad(angle_deg);//publishes new angle
	  publish_joint_ind();
	  ros::Duration(2).sleep();

	  currentPose = rad_to_deg( joint_positions->at(p_id) );
	  if(counter == 0){
	    lastPose = currentPose;
	    minvalue = currentPose;
	  }

	  ROS_INFO("   currentPose: %f lastPose: %f", currentPose, lastPose);
	  ROS_INFO("   desired angle: %f ID: %d", angle_deg, p_id);

	  double posediff = fabs(currentPose - lastPose);

	  if(currentPose > lastPose && posediff > diff_thresh){
	    ROS_INFO("   joint turned!!**");
	    if(lastPose < minvalue)
	      minvalue = lastPose;
	    break;
	  }
	  //if(currentPose > lastPose && posediff < diff_thresh){
	  //  minvalue = lastPose;
	  //  break;
	  //}
	  //if(currentPose < lastPose && posediff < diff_thresh){
	  //  minvalue = currentPose;
	  //  break;
	  //}
	  if(counter > 3 && (fabs(angle_deg-currentPose) > p_angle_increment_deg*1.5) && (posediff < diff_thresh)){
	    ROS_INFO("   joint stalled!!**");
	    if(currentPose < minvalue)
	      minvalue = currentPose;
	    break;
	  }

	  if(currentPose < minvalue)
	    minvalue = currentPose;

	  counter++;
	  lastPose = currentPose;
	  ros::spinOnce();
	  
	}
	ROS_INFO("Found min value!: %f ID: %d", minvalue, p_id);

	minAdjust(minvalue,p_id,final_min);

	ROS_INFO("Final adjusted min value: %f ID: %d", final_min, p_id);
}

#if 0
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
	bool checker_sum = false;
	
	while(continueCalibration && ros::ok())
	{	
	  checker_sum = false;
	  angle_deg += direction[p_id] * p_angle_increment_deg;
	  desiredJointPos->angle = deg_to_rad(angle_deg);//publishes new angle
	  publish_joint_ind();
	  ros::Duration(2).sleep();
		
	  currentPose = rad_to_deg( joint_positions->at(p_id) );//stores the value at the p_id-th position in array joint_position to currentPose

	  ROS_INFO("currentPose: %f lastPose: %f", currentPose, lastPose);
	  ROS_INFO("desired angle: %f ID: %d", angle_deg, p_id);

	  double posediff = fabs(currentPose - lastPose);
		
	  // Checks if the Joint Angle has not changed or has gone in the reverse direction
	  if (direction[p_id] == -1){
	    if(posediff < 5.0){
	      if(currentPose < lastPose){
		
	      }
	    }
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
#endif

/*
 * This function is used to set the offset for the physical hardware calibration limits for Bandit II
 * First run bandit_calib_version1 and use the service call to determine the min/max angle of the joints and input below
 * The Second value of (XXX + XX) is measure for the zero position of hands down, palm facing inwards, and forearm facing outwards
 */

void true_zero(const int & id, double max, double min)
{	
  double ideal_max[19] = {0,0,197,169,175,115,90,0,0,197,169,175,115,0,0,0,0,0,0};
  double ideal_min[19] = {0,0,-93,3,-130,0,-77,0,0,-93,3,-130,0,-77,0,0,0,0,0};
  double offset_max, offset_min, ideal_diff, encoder_diff;

  if(id >= 2 && id <= 13){
    offset_max = max - direction_Emit[id]*ideal_max[id];
    offset_min = min - direction_Emit[id]*ideal_min[id];
    
    truezero = (offset_max + offset_min)/2.0;

    ideal_diff = ideal_max[id] - ideal_min[id];
    encoder_diff = max - min;

    ROS_INFO("Joint Range [%d]: %f - Ideal: %f - Diff: %f\n",id,encoder_diff,ideal_diff,(fabs(ideal_diff - encoder_diff)));
  }else{
    ROS_INFO("No Joint Calibration Available");
    truezero = 0.0;
  }	
}

#if 0
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
#endif

int main( int argc, char* argv[] )
{
	ros::init(argc,argv,"bandit_calib");
	
	ros::NodeHandle n;
	joint_publisher = new ros::Publisher;
	*joint_publisher = n.advertise<bandit_msgs::JointArray>("joint_cmd",5); //creates publisher and publishes to topic "joint_ind"
	
	joint_subscriber = new ros::Subscriber;
	*joint_subscriber = n.subscribe("joint_states", 5, jointStatesCallBack); //creates subscriber and subscribes to topic "joint_states"

	spinner = new ros::AsyncSpinner(2);//Use 2 threads
	ros::Duration(1).sleep();
  spinner->start();//starts threads
	desiredJointPos = new bandit_msgs::Joint;
	
	joint_positions = new std::vector<double>;
	
	std::string filename = "Bandit_Calibration_File.yaml";
	ofstream write_bandit_file;//declares output stream
	write_bandit_file.open(filename.c_str());//creates file Bandit_Calibration_File.yaml
	YAML::Emitter emitter;//declares output emitter for yaml formatting
	emitter << YAML::BeginSeq;

	double max_joint = 0, min_joint = 0;
	
	if(ros::ok()){
		//calibrates each joint
		for ( int z_id = 0; z_id < 19; z_id++){
		  if(z_id == 0 || z_id == 1 || z_id == 7 || z_id == 8 || z_id >= 14){
		    truezero = 0.0;
		    offset = 0.0;
		    max_joint = 0.0;
		    min_joint = 0.0;
		  }
		  else{
			if(z_id == 3 || z_id == 4 || z_id == 10 || z_id == 11){
			  calibrateJoint(z_id, 30, position_zero, max_joint, min_joint);
			  true_zero(z_id, max_joint, min_joint);
			}
			else if(z_id == 2 || z_id == 9){
			  calibrateJoint(z_id, 20, position_zero, max_joint, min_joint);
			  true_zero(z_id, max_joint, min_joint);
			}
			else {
			  calibrateJoint(z_id, 10, position_zero, max_joint, min_joint);
			  true_zero(z_id, max_joint, min_joint);				
			}
			ROS_INFO("True Zero: %f - Max: %f - Min: %f\n",truezero,max_joint,min_joint);
		  }
			//output values for the YAML File
			emitter << YAML::BeginMap;
			emitter << YAML::Key << "JointID" << YAML::Value << z_id;
			emitter << YAML::Key << "Direction" << YAML::Value << direction_Emit[z_id];
			emitter << YAML::Key << "TrueZero" << YAML::Value << truezero;
			emitter << YAML::Key << "Offset" << YAML::Value << offset;
			emitter << YAML::Key << "MaxAngle" << YAML::Value << max_joint;
			emitter << YAML::Key << "MinAngle" << YAML::Value << min_joint;
			emitter << YAML::EndMap;
			//std::cout << emitter.c_str() << endl;	
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
