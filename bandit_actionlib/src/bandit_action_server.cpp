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
#include <sstream>
#include <yaml-cpp/yaml.h>

using namespace std;

/*
 * Initialize Objects
 */
bandit_msgs::JointArray * jarray;
ros::Publisher * joint_publisher;
ros::Subscriber * joint_subscriber;
ros::AsyncSpinner * spinner;

std::vector<double> * joint_positions;

/*
 * Changes degrees to radians and outputs radians
 */
double deg_to_rad(double deg)
{
	return deg * M_PI / 180;
}

/*
 * Changes radians to degrees and outputs degrees
 */
double rad_to_deg(double rad)
{
	return rad * 180 / M_PI;
}

/*
 * Publishes the joint array: jarray with values for joint id and joint angle
 */
void publish_joint_ind()
{
	joint_publisher->publish(*jarray);
}

/*
 * Subscribes to joint_states and takes the information from encoder and stores it to joint_positions
 */
void jointStatesCallBack(const sensor_msgs::JointStateConstPtr &js)
{
	*joint_positions = js->position;//pulls out all the positions for all joints and stores it to joint_positions
}

/*
 * Initialize data structure Joints for parsing in YAML file
 */
struct Joints {
	float id, joint_angle;
};

/*
 * Initialize data structure Frame for parsing in YAML file
 */
struct Frame {
	int frame_num;
	int hold_time;
	std::vector <Joints> joints;
};

/*
 * operator for storing Joint Id and Joint Angle into object Joint
 */
void operator >> (const YAML::Node& node, Joints& joint){
	node["Joint ID"] >> joint.id;
	node["Joint Angle"] >> joint.joint_angle;
}

/*
 * operator for storing Frame Num, Hold Time, and Joint into object Frame
 */
void operator >> (const YAML::Node& node, Frame& frame){
	node["Frame Num"] >> frame.frame_num;
	node["Hold Time"] >> frame.hold_time;
	const YAML::Node& joints = node["Joints"];
	for (unsigned i=0; i<joints.size();i++){
		Joints joint;
		joints[i] >> joint;
		frame.joints.push_back(joint);
	}
}

/*
 * Creates class BanditAction
 */
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
		/*
		 * Constructors for BanditAction
		 */
		BanditAction(std::string name) :
			as (n, name, boost::bind(&BanditAction::executeCB, this, _1)), action_name(name)
		{
		}
		
		~BanditAction(void)
		{
		}

	/*
	 * Whenever BanditAction Class is created this function is called, it executes the gestures on the Bandit II
	 */
	void executeCB ( const bandit_actionlib::BanditGoalConstPtr &goal )
	{
		bool success = true;
		double current_time = 0;
		double first_time = ros::Time::now().toSec();
		joint_publisher = new ros::Publisher;
		*joint_publisher = n.advertise<bandit_msgs::JointArray>("joint_cmd",5);//this topic can be changed to "joints" to actuate fake_bandit
		joint_subscriber = new ros::Subscriber;
		*joint_subscriber = n.subscribe("joint_states", 10, jointStatesCallBack); //creates subscriber and subscribes to topic "joint_states"
		
		bandit_msgs::Joint desired_joint_pos;
		jarray = new bandit_msgs::JointArray; //creates a new JointArray, jarray
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
		YAML::Parser parser(fin);
		YAML::Node doc;
		parser.GetNextDocument(doc);
		
		while(success){
			//set preempt
			if (as.isPreemptRequested() || !ros::ok()){
				ROS_INFO("%s: Preempted", action_name.c_str());
				// set the action state to preempted
				as.setPreempted();
				success = false;
				break;
			}
			
			/* reads parsed YAML file and assigns gesture values to bandit_msgs::Joint desired_joint_pos
			 * takes the desired_joint_pos and puts them into bandit_msgs::JointArray jarray
			 * publishes feedback about which angles and joints are being slotted into the array as well as the time
			 */
			for(unsigned k=0;k < doc.size();k++) {
				Frame frames;
				doc[k] >> frames;
				for (int l =0; l < frames.joints.size(); l++){
					Joints g = frames.joints[l];
					desired_joint_pos.id = g.id;
					desired_joint_pos.angle = deg_to_rad(g.joint_angle);
					jarray->joints.push_back(desired_joint_pos);
					//publishes feedback information
					current_time = ros::Time::now().toSec(); //gets the current time
					feedback.progress_time = current_time;
					feedback.progress_hold_time = frames.hold_time;
					feedback.progress_joint_id.push_back( g.id );
					feedback.progress_joint_angle.push_back( g.joint_angle );
					as.publishFeedback(feedback); //publishes the feedback information
					ros::Rate(5).sleep();
				}
				//publishes joint array for each frame (note: JointArray is normally set to radians)
				publish_joint_ind();
				ros::spinOnce();
				ros::Rate(1).sleep();
				ros::Duration(frames.hold_time).sleep(); // forces the node to sleep for the hold_time
			}
			
			//publishes scucessful result
		    if(success){
				result.total_time = current_time - first_time;
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

  BanditAction bandit_action(ros::this_node::getName());//creates BanditAction object
  ros::spin();

  return 0;
}
