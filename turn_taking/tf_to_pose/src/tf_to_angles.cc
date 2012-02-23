#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>

#ifndef PI
#define PI 3.14159265359
#endif
#ifndef HALFPI
#define HALFPI 1.57079632679
#endif
#ifndef QUARTPI
#define QUARTPI 0.785398163397
#endif

#define TORSO 						0
#define LEFT_SHOULDER 		1
#define LEFT_ELBOW 				2
#define LEFT_HAND 				3
#define RIGHT_SHOULDER 		4
#define RIGHT_ELBOW 			5
#define RIGHT_HAND 				6
#define NECK							7

KDL::Vector left_hand;
KDL::Vector left_elbow;
KDL::Vector left_shoulder;
KDL::Vector right_hand;
KDL::Vector right_elbow;
KDL::Vector right_shoulder;
KDL::Vector neck;
KDL::Vector torso;
tf::TransformListener* tl;
std::string global_frame;
std::string suffix;

ros::Time time_;

void set_vector( tf::StampedTransform t, KDL::Vector& v )
{
	v.x(t.getOrigin().x());
	v.y(t.getOrigin().y());
	v.z(t.getOrigin().z());
}

bool get_poses()
{
	try{
		tf::StampedTransform stf;
		tl->lookupTransform( global_frame, "torso"+suffix, ros::Time(0), stf );
		set_vector( stf, torso );
		tl->lookupTransform( global_frame, "left_shoulder"+suffix, ros::Time(0), stf );
		set_vector( stf, left_shoulder );
    tl->lookupTransform( global_frame, "left_elbow"+suffix, ros::Time(0), stf );
		set_vector( stf, left_elbow );
    tl->lookupTransform( global_frame, "left_hand"+suffix, ros::Time(0), stf );
		set_vector( stf, left_hand );
		tl->lookupTransform( global_frame, "right_shoulder"+suffix, ros::Time(0), stf );
		set_vector( stf, right_shoulder );
		tl->lookupTransform( global_frame, "right_elbow"+suffix, ros::Time(0), stf );
		set_vector( stf, right_elbow );
    tl->lookupTransform( global_frame, "right_hand"+suffix, ros::Time(0), stf );
		set_vector( stf, right_hand );
    tl->lookupTransform( global_frame, "neck"+suffix, ros::Time(0), stf );
		set_vector( stf, neck );
		time_ = stf.stamp_;
/*
		tl->lookupTransform( prefix + "torso", global_frame, ros::Time(0), stf );
		set_vector( stf, torso );
		tl->lookupTransform( prefix + "left_shoulder", global_frame, ros::Time(0), stf );
		set_vector( stf, left_shoulder );
		tl->lookupTransform( prefix + "left_elbow", global_frame, ros::Time(0), stf );
		set_vector( stf, left_elbow );
		tl->lookupTransform( prefix + "left_hand", global_frame, ros::Time(0), stf );
		set_vector( stf, left_hand );
		tl->lookupTransform( prefix + "right_shoulder", global_frame, ros::Time(0), stf );
		set_vector( stf, right_shoulder );
		tl->lookupTransform( prefix + "right_elbow", global_frame, ros::Time(0), stf );
		set_vector( stf, right_elbow );
		tl->lookupTransform( prefix + "right_hand", global_frame, ros::Time(0), stf );
		set_vector( stf, right_hand );
		tl->lookupTransform( prefix + "neck", global_frame, ros::Time(0), stf );
		set_vector( stf, neck );
*/
}
	catch( tf::TransformException &ex )
	{
		ROS_WARN( "could not do transform: [%s]", ex.what() );
		return false;
	}

	return true;
}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "tf_to_angles" );
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	tl = new tf::TransformListener();
	global_frame = "camera_skeleton_frame";
	suffix = "";

  ros::Publisher joint_pub_ = nh.advertise<sensor_msgs::JointState>("output_joint_state",10);


	while( ros::ok() )
	{
		if( !get_poses() ) 
		{
			loop_rate.sleep();
			ros::spinOnce();
			continue;
		}
		// for each end effector
    // left elbow roll
    KDL::Vector left_elbow_hand(left_hand - left_elbow);
    KDL::Vector left_elbow_shoulder(left_shoulder - left_elbow);
    left_elbow_hand.Normalize();
    left_elbow_shoulder.Normalize();
    static double left_elbow_angle_roll = 0;
    //if (joint_position_left_hand.fConfidence >= 0.5 &&
    //    joint_position_left_elbow.fConfidence >= 0.5 &&
    //    joint_position_left_shoulder.fConfidence >= 0.5)
    {
      left_elbow_angle_roll = acos(KDL::dot(left_elbow_hand, left_elbow_shoulder));
      left_elbow_angle_roll = left_elbow_angle_roll - PI;
    }

    // right elbow roll
    KDL::Vector right_elbow_hand(right_hand - right_elbow);
    KDL::Vector right_elbow_shoulder(right_shoulder - right_elbow);
    right_elbow_hand.Normalize();
    right_elbow_shoulder.Normalize();
    static double right_elbow_angle_roll = 0;
    //if (joint_position_right_hand.fConfidence >= 0.5 &&
    //    joint_position_right_elbow.fConfidence >= 0.5 &&
    //    joint_position_right_shoulder.fConfidence >= 0.5)
    {
      right_elbow_angle_roll = acos(KDL::dot(right_elbow_hand, right_elbow_shoulder));
      right_elbow_angle_roll = -(right_elbow_angle_roll - PI);
    }

    // left shoulder roll
    KDL::Vector left_shoulder_elbow(left_elbow - left_shoulder);
    KDL::Vector left_shoulder_neck(neck - left_shoulder);
    left_shoulder_elbow.Normalize();
    left_shoulder_neck.Normalize();
    static double left_shoulder_angle_roll = 0;
    //if (joint_position_neck.fConfidence >= 0.5 &&
    //    joint_position_left_elbow.fConfidence >= 0.5 &&
    //    joint_position_left_shoulder.fConfidence >= 0.5)
    {
      left_shoulder_angle_roll = acos(KDL::dot(left_shoulder_elbow, left_shoulder_neck));
      left_shoulder_angle_roll = left_shoulder_angle_roll - HALFPI;
    }

    // right shoulder roll
    KDL::Vector right_shoulder_elbow(right_elbow - right_shoulder);
    KDL::Vector right_shoulder_neck(neck - right_shoulder);
    right_shoulder_elbow.Normalize();
    right_shoulder_neck.Normalize();
    static double right_shoulder_angle_roll = 0;
    //if (joint_position_neck.fConfidence >= 0.5 &&
    //    joint_position_right_elbow.fConfidence >= 0.5 &&
    //    joint_position_right_shoulder.fConfidence >= 0.5)
    {
      right_shoulder_angle_roll = acos(KDL::dot(right_shoulder_elbow, right_shoulder_neck));
      right_shoulder_angle_roll = -(right_shoulder_angle_roll - HALFPI);
    }

    // left shoulder pitch
    static double left_shoulder_angle_pitch = 0;
    //if (joint_position_left_shoulder.fConfidence >= 0.5)
    {
      left_shoulder_angle_pitch = asin(left_shoulder_elbow.y());
      left_shoulder_angle_pitch = left_shoulder_angle_pitch + HALFPI;
    }

    // right shoulder pitch
    static double right_shoulder_angle_pitch = 0;
    //if (joint_position_right_shoulder.fConfidence >= 0.5)
    {
      right_shoulder_angle_pitch = asin(right_shoulder_elbow.y());
      right_shoulder_angle_pitch = -(right_shoulder_angle_pitch + HALFPI);
    }

    // left shoulder yaw
    static double left_shoulder_angle_yaw = 0;
    //if (joint_position_left_shoulder.fConfidence >= 0.5)
    {
      left_shoulder_angle_yaw = asin(left_elbow_hand.y());  // left_shoulder_elbow.x()
    }

    // right shoulder yaw
    static double right_shoulder_angle_yaw = 0;
    //if (joint_position_right_shoulder.fConfidence >= 0.5)
    {
      right_shoulder_angle_yaw = asin(right_elbow_hand.y());  // left_shoulder_elbow.x()
      right_shoulder_angle_yaw = -right_shoulder_angle_yaw;
    }

		left_shoulder_angle_yaw -= M_PI/2.;
		while( left_shoulder_angle_yaw < -M_PI ) left_shoulder_angle_yaw += 2*M_PI;
		right_shoulder_angle_yaw = -right_shoulder_angle_yaw - M_PI/2.;
		while( right_shoulder_angle_yaw < -M_PI ) right_shoulder_angle_yaw += 2*M_PI;

		// get joint angles from positions
		sensor_msgs::JointState js;
		js.header.stamp = time_;
		js.name.push_back("left_bicep_forearm_joint");
		js.position.push_back(-left_elbow_angle_roll);
		js.velocity.push_back(0);
		js.effort.push_back(0);
		js.name.push_back("left_shoulder_mounting_shoulder_joint");
		js.position.push_back(left_shoulder_angle_roll);
		js.velocity.push_back(0);
		js.effort.push_back(0);
		js.name.push_back("left_torso_shoulder_mounting_joint");
		js.position.push_back(left_shoulder_angle_pitch);
		js.velocity.push_back(0);
		js.effort.push_back(0);
		js.name.push_back("left_shoulder_bicep_joint");
		js.position.push_back(left_shoulder_angle_yaw);
		js.velocity.push_back(0);
		js.effort.push_back(0);

		js.name.push_back("right_bicep_forearm_joint");
		js.position.push_back(right_elbow_angle_roll);
		js.velocity.push_back(0);
		js.effort.push_back(0);
		js.name.push_back("right_shoulder_mounting_shoulder_joint");
		js.position.push_back(-right_shoulder_angle_roll);
		js.velocity.push_back(0);
		js.effort.push_back(0);
		js.name.push_back("right_torso_shoulder_mounting_joint");
		js.position.push_back(-right_shoulder_angle_pitch);
		js.velocity.push_back(0);
		js.effort.push_back(0);
		js.name.push_back("right_shoulder_bicep_joint");
		js.position.push_back(right_shoulder_angle_yaw);
		js.velocity.push_back(0);
		js.effort.push_back(0);

		joint_pub_.publish(js);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
