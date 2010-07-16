#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <bandit_msgs/JointArray.h>
#include <bandit_msgs/Params.h>
#include <math.h>
#include <bandit/bandit.h>

using namespace std;
bandit::Bandit g_bandit;
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

void bandit_nod()
{	
	double nod_deg = 15;
	desired_joint_pos->id = 0;
	for (int i_nod = 0; i_nod < 8; i_nod++){
		if (i_nod % 2 == 0){
			nod_deg = - nod_deg;
		}
		desired_joint_pos->angle = deg_to_rad( nod_deg );
		publish_joint_ind();
		ros::Rate(5).sleep();
	}
	desired_joint_pos->angle = deg_to_rad(0);
	publish_joint_ind();
	ros::Rate(1).sleep();
}

void bandit_shake()
{
	double shake_deg = 30;
	desired_joint_pos->id = 1;
	for (int i_shake = 0; i_shake < 8; i_shake++){
		if (i_shake % 2 == 0){
			shake_deg = - shake_deg;
		}
		desired_joint_pos->angle = deg_to_rad( shake_deg );
		publish_joint_ind();
		ros::Rate(5).sleep();
	}
	desired_joint_pos->angle = deg_to_rad(0);
	publish_joint_ind();
	ros::Rate(1).sleep();
}

void bandit_wave()
{
	desired_joint_pos->id = 9;
	desired_joint_pos->angle = deg_to_rad(90);
	publish_joint_ind();
	ros::Rate(10).sleep();
	
	desired_joint_pos->id = 10;
	desired_joint_pos->angle = deg_to_rad(90);
	publish_joint_ind();
	ros::Rate(10).sleep();
	
	desired_joint_pos->id = 12;
	desired_joint_pos->angle = deg_to_rad(90);
	publish_joint_ind();
	ros::Rate(10).sleep();
	
	ros::Duration(0.5).sleep();
	
	double wave_deg = 0;
	desired_joint_pos->id = 12;
	for (int i_wave = 0; i_wave < 12; i_wave++){
		if (i_wave % 2 == 0){
			wave_deg = 60;
		}
		else
			wave_deg = 110;
		desired_joint_pos->angle = deg_to_rad( wave_deg );
		publish_joint_ind();
		ros::Rate(2).sleep();
	}
	
	for (int ii = 0; ii < 16; ii++){
		desired_joint_pos->id = ii;
		if (ii == 5){
			desired_joint_pos->angle = deg_to_rad(10);
		}
		else if (ii == 7 || ii == 8 || ii == 14 || ii == 15){
			continue;
		}
		else
			desired_joint_pos->angle = deg_to_rad( 0 );
		publish_joint_ind();
		ros::Rate(10).sleep();
	}
}

void bandit_comehere()
{
	desired_joint_pos->id = 9;
	desired_joint_pos->angle = deg_to_rad(60);
	publish_joint_ind();
	ros::Rate(10).sleep();
	
	desired_joint_pos->id = 12;
	desired_joint_pos->angle = deg_to_rad(115);
	publish_joint_ind();
	ros::Rate(10).sleep();
	
	desired_joint_pos->id = 13;
	desired_joint_pos->angle = deg_to_rad(90);
	publish_joint_ind();
	ros::Rate(10).sleep();
	
	ros::Duration(0.5).sleep();

	double comehere_deg = 0;
	desired_joint_pos->id = 12;
	for (int i_comehere = 0; i_comehere < 8; i_comehere++){
		if (i_comehere % 2 == 0){
			comehere_deg = 90;
		}
		else
			comehere_deg = 115;
		desired_joint_pos->angle = deg_to_rad( comehere_deg );
		publish_joint_ind();
		ros::Rate(2).sleep();
	}
	
		for (int i_i = 0; i_i < 16; i_i++){
		desired_joint_pos->id = i_i;
		if (i_i == 5){
			desired_joint_pos->angle = deg_to_rad(10);
		}
		else if (i_i == 7 || i_i == 8 || i_i == 14 || i_i == 15){
			continue;
		}
		else
			desired_joint_pos->angle = deg_to_rad( 0 );
		publish_joint_ind();
		ros::Rate(10).sleep();
	}
}

int main( int argc, char* argv[] )
{
	ros::init(argc,argv,"bandit_calib");
	
	ros::NodeHandle n;
	
	joint_publisher = new ros::Publisher;
	*joint_publisher = n.advertise<bandit_msgs::Joint>("joint_ind",10);
	
	joint_subscriber = new ros::Subscriber;
	*joint_subscriber = n.subscribe("joint_states", 10, jointStatesCallBack); //creates subscriber and subscribes to topic "joint_states"

	spinner = new ros::AsyncSpinner(2);//Use 2 threads
	ros::Rate(1).sleep();
	
	spinner->start();
	
	desired_joint_pos = new bandit_msgs::Joint;
	
	joint_positions = new std::vector<double>;
	
	while(ros::ok()){
		bandit_wave();
		ros::Duration(1).sleep();
		bandit_nod();
		ros::Duration(1).sleep();
		bandit_comehere();
		ros::Duration(1).sleep();
		bandit_shake();
		ros::Duration(1).sleep();
	}
	
	return 0;
}
