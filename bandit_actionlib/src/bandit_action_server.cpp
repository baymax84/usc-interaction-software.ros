#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <bandit_actionlib/BanditAction.h>
#include <sensor_msgs/JointState.h>
#include <bandit_msgs/JointArray.h>
#include <bandit_msgs/Params.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <sstream>
#include <ctime>
#include <yaml-cpp/yaml.h>

using namespace std;
bandit_msgs::Joint * desired_joint_pos;
ros::Publisher * joint_publisher;
ros::Subscriber * joint_subscriber;
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

void publish_joint_ind()
{
	joint_publisher->publish(*desired_joint_pos);
}

void jointStatesCallBack(const sensor_msgs::JointStateConstPtr &js)
{
	*joint_positions = js->position;//pulls out all the positions for all joints and stores it to joint_positions
}

struct Joint_Calibrations {
	float id, joint_angle;
};

void operator >> (const YAML::Node& node, Joint_Calibrations& gesture){
	node["Joint ID"] >> gesture.id;
	node["Joint Angle"] >> gesture.joint_angle;
}

class BanditAction
{
	
	protected:

		ros::NodeHandle n;
		actionlib::SimpleActionServer<bandit_actionlib::BanditAction> as;
		std::string action_name;
		// create messages that are used to published feedback/result
		bandit_actionlib::BanditFeedback feedback;
		bandit_actionlib::BanditResult result;

	public:

		BanditAction(std::string name) :
			as (n, name, boost::bind(&BanditAction::executeCB, this, _1)), action_name(name)
		{
		}

		~BanditAction(void)
		{
		}

	void executeCB ( const bandit_actionlib::BanditGoalConstPtr &goal )
	{
		bool success = true;
		time_t seconds;
		seconds = time(NULL);
		joint_publisher = new ros::Publisher;
		*joint_publisher = n.advertise<bandit_msgs::Joint>("joint_ind",10);
		joint_subscriber = new ros::Subscriber;
		*joint_subscriber = n.subscribe("joint_states", 10, jointStatesCallBack); //creates subscriber and subscribes to topic "joint_states"
		
		desired_joint_pos = new bandit_msgs::Joint;
		joint_positions = new std::vector<double>;
		
		spinner = new ros::AsyncSpinner(2);//Use 2 threads
		ros::Rate(1).sleep();
		spinner->start();
		
		feedback.progress_joint_id.clear();
		feedback.progress_joint_angle.clear();
		feedback.progress_joint_id.push_back(0);
		feedback.progress_joint_angle.push_back(0);
		
		// publish info to the console for the user
		ROS_INFO("%s: Executing, Bandit Server Running. Ready to Recieve YAML files.", action_name.c_str());

		//parses incoming YAML file
		std::ifstream fin;
		const std::string fileName = goal->frame;
		fin.open(fileName.c_str());
		if (fin.fail()){
			ROS_WARN("Failure to find File");
		}
		YAML:: Parser parser(fin);
		YAML::Node doc;
		parser.GetNextDocument(doc);
		Joint_Calibrations gesture;
		//initalize dynamic arrays for ID and corresponding angle
		int* j_id = NULL;
		int j_id_size;
		double* j_angle = NULL;
		int j_angle_size;
		j_id_size = doc.size();
		j_angle_size = doc.size();
		j_id = new int [j_id_size];
		j_angle = new double [j_angle_size];

		while(success){
			//set preempt
			if (as.isPreemptRequested() || !ros::ok()){
				ROS_INFO("%s: Preempted", action_name.c_str());
				// set the action state to preempted
				as.setPreempted();
				success = false;
				break;
			}
			
			//reads parsed YAML file and assigns gesture values to j_cal
			for(unsigned k=0;k < doc.size();k++) {
				doc[k] >> gesture;
				j_id[k] = gesture.id;
				j_angle[k] = gesture.joint_angle;
			}
			//publishes each joint id and corresponding angle to bandit_node and publishes feedback
			for (unsigned jj= 0; jj < doc.size(); jj++){
				desired_joint_pos->id = j_id[jj];
				desired_joint_pos->angle = deg_to_rad( j_angle[jj] );
				publish_joint_ind();
				if (jj != 0){
					if (fabs(j_angle[jj] - j_angle[jj - 1]) >= 60){
						ros::Rate(10).sleep();
					}
					else
						ros::Rate(5).sleep();
				}
				feedback.progress_time = seconds;
				feedback.progress_joint_id.push_back( j_id[jj] );
				feedback.progress_joint_angle.push_back( j_angle[jj] );
				as.publishFeedback(feedback);
				ros::Rate(10).sleep();
			}
			//publishes scucessful result
		    if(success){
				result.total_time = feedback.progress_time;
				result.result_joint_id = feedback.progress_joint_id;
				result.result_joint_angle = feedback.progress_joint_angle;
				ROS_INFO("%s: Succeeded", action_name.c_str());
				// set the action state to succeeded
				as.setSucceeded(result);
				success = false;
			}
			
		}
	}
	
	
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bandit_action");

  BanditAction bandit_action(ros::this_node::getName());
  ros::spin();

  return 0;
}
