#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <std_msgs/String.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "intial_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("hi", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);


    // robot state
		double HeadNod=0, HeadTurn =0, Mouth=0, EyesLtRt=0, 
		EyeBlink=0, LtArmOut=0, LtArmForward=0, RtArmForward=0,
		RtArmOut=0, LtElbow=0, RtElbow=0, LtWrist=0, RtWrist=0,
		LtFootForward=0, RtFootForward=0, LtFootUp=0, RtFootUp=0,
		TorsoBend=0, angle=0;
		
    // message declarations
 geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "world";


    while (ros::ok()) {
		joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(18);
        joint_state.position.resize(18);
        
        //head
		joint_state.name[0] ="HeadNod";
        joint_state.position[0] = HeadNod;
		joint_state.name[1] ="HeadTurn";
		joint_state.position [1] =HeadTurn;        
        joint_state.name[2] ="Mouth";
        joint_state.position[2] =Mouth;
        
        //eyes
        joint_state.name[3] ="EyesLtRt";
        joint_state.position[3] =EyesLtRt;
		joint_state.name[4] ="EyeBlink";
		joint_state.position [4] =EyeBlink;
		
		//left arm              
        joint_state.name[5] ="LtArmOut";
        joint_state.position[5] =LtArmOut;
		joint_state.name [6]="LtArmForward";
		joint_state.position[6]=LtArmForward;        
		joint_state.name[7] ="LtElbow";
		joint_state.position [7] =LtElbow;        
		joint_state.name[8] ="LtWrist";
		joint_state.position [8] =LtWrist;
		
		//right arm
		joint_state.name[9] ="RtArmOut";
        joint_state.position[9] =RtArmOut;		        
        joint_state.name[10] ="RtArmForward";
        joint_state.position[10] =RtArmForward;
		joint_state.name[11] ="RtElbow";
		joint_state.position [11] =RtElbow;
		joint_state.name[12] ="RtWrist";
		joint_state.position [12] =RtWrist;
		
		//left leg
		joint_state.name[13] ="LtFootForward";
		joint_state.position [13] =LtFootForward;
		joint_state.name[14] ="LtFootUp";
		joint_state.position [14] =LtFootUp;		
		
		//right leg
		joint_state.name[15] ="RtFootForward";
		joint_state.position [15] =RtFootForward;
		joint_state.name[16] ="RtFootUp";
		joint_state.position [16] =RtFootUp;
		
		//spine
		joint_state.name[17] ="TorsoBend";
		joint_state.position [17] =TorsoBend;






 // update transform
        
 
        //send the joint state and transform
         // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle)*2;
        odom_trans.transform.translation.y = sin(angle)*2;
        odom_trans.transform.translation.z = .7;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform

        joint_pub.publish(joint_state);        
        broadcaster.sendTransform(odom_trans);
	ros::spinOnce();
        // Create new robot state
    //  uppercenterjoint -= tinc;
     //   if (uppercenterjoint<-.7 || uppercenterjoint>0) tinc *= -1;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
