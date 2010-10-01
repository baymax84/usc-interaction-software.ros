
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace ros;

// global variables
ros::Publisher g_joint_pub;

/*class JointBroadcaster
{
  public:
    JointBroadcaster()
    {
		joint_pub_ = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	};

    void jointCallback(...);

  private:
    ros::Publisher joint_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
};*/ 

void jointCallback(const sensor_msgs::JointStateConstPtr& state)
{
	geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "world";



		double 	 HeadNod, Mouth, EyesLtRt, EyeBlink, LtArmOut, 
		LtArmForward, RtArmForward, RtArmOut, LtElbow, RtElbow, LtWrist,
		RtWrist, LtFootForward, RtFootForward, LtFootUp, RtFootUp, 
		TorsoBend, HeadTurn,  angle=0;

		float radcorrect=3.14159f/180.0f;

		joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(64);
        joint_state.position.resize(64);
        
        
         ROS_INFO("running ok");
             
        //head
        joint_state.name[0] ="Mouth";
        Mouth = radcorrect*state->position[0];
        joint_state.position[0] = Mouth;
		joint_state.name[1] ="HeadNod";
		HeadNod = radcorrect*state->position[1];
        joint_state.position[1] = HeadNod;
		joint_state.name [2] ="HeadTurn";
		HeadTurn= radcorrect*state->position[2];
		joint_state.position [2] =HeadTurn;
		
		
		//eyes
        joint_state.name[3] ="EyesLtRt";
        EyesLtRt = radcorrect*state->position[13];
        joint_state.position[3] =EyesLtRt;
		joint_state.name [4] ="LeyeJoint";
		joint_state.position [4] =EyesLtRt;        
		joint_state.name[5] ="EyeBlink";
		EyeBlink = radcorrect*(90-state->position[12]);
		joint_state.position [5] =-EyeBlink;
		joint_state.name[6] ="RLidJoint";
		joint_state.position [6] =-EyeBlink;		
		joint_state.name[7] ="LPinJoint";
		joint_state.position[7] = -1*(EyesLtRt);
		joint_state.name[8] ="EyeBeamJoint";
		joint_state.position[8]=.5113*(EyesLtRt);
		joint_state.name[9] ="EyeBallPinJoint";
		joint_state.position[9]=(EyesLtRt);
		joint_state.name[10] ="EyeBallPinJoint2";
		joint_state.position[10]=(EyesLtRt);
		
		//left arm
        joint_state.name[11] ="LtArmOut";
        LtArmOut = radcorrect*state->position[7];
        joint_state.position[11] =LtArmOut;
		joint_state.name [12]="LtArmForward";
		LtArmForward = radcorrect*state->position[6];
		joint_state.position[12]=LtArmForward;
		joint_state.name[13] ="LtElbow";
		LtElbow = radcorrect*state->position[8];
		joint_state.position [13] =-LtElbow;		 
		joint_state.name[14] ="LtWrist";
		LtWrist = radcorrect*state->position[10];
		joint_state.position [14] =LtWrist;
		
		//right arm		       
 		joint_state.name[15] ="RtArmOut";
		RtArmOut = radcorrect*state->position[4];
        joint_state.position[15] =-RtArmOut;       
        joint_state.name[16] ="RtArmForward";
        RtArmForward = radcorrect*state->position[3];
        joint_state.position[16] =RtArmForward;
		joint_state.name[17] ="RtElbow";
		RtElbow = radcorrect*state->position[5];
		joint_state.position [17] =RtElbow;
		joint_state.name[18] ="RtWrist";
		RtWrist = radcorrect*state->position[9];
		joint_state.position [18] =-RtWrist;

		//left leg
		joint_state.name[19] ="LtFootForward";
		LtFootForward = radcorrect*state->position[17];
		joint_state.position [19] =LtFootForward;
		joint_state.name[20] ="LtFootUp";
		LtFootUp = radcorrect*state->position[16];
		joint_state.position [20] =-LtFootUp;
		
		//right leg
		joint_state.name[21] ="RtFootForward";
		RtFootForward = radcorrect*state->position[15];
		joint_state.position [21] =RtFootForward;
		joint_state.name[22] ="RtFootUp";
		RtFootUp = radcorrect*state->position[14];
		joint_state.position [22] =-RtFootUp;
		
		//upper spine
		joint_state.name [23] ="TorsoBend";
		TorsoBend= radcorrect*state->position[11];
		joint_state.position [23] =TorsoBend/41;
		joint_state.name[24] ="TSJ2";
		joint_state.position [24] =-(TorsoBend/41);
		joint_state.name[25] ="TSJ3";
		joint_state.position [25] =-(TorsoBend/41);
		joint_state.name[26] ="TSJ4";
		joint_state.position [26] =-(TorsoBend/41);
		joint_state.name[27] ="TSJ5";
		joint_state.position [27] =-(TorsoBend/41);	
		joint_state.name[28] ="TSJ6";
		joint_state.position [28] =-(TorsoBend/41);
		joint_state.name[29] ="TSJ7";
		joint_state.position [29] =-(TorsoBend/41);
		joint_state.name[30] ="TSJ8";
		joint_state.position [30] =-(TorsoBend/41);
		joint_state.name[31] ="TSJ9";
		joint_state.position [31] =-(TorsoBend/41);
		joint_state.name[32] ="TSJ10";
		joint_state.position [32] =-(TorsoBend/41);
		joint_state.name[33] ="TSJ11";
		joint_state.position [33] =-(TorsoBend/41);
		joint_state.name[34] ="TSJ12";
		joint_state.position [34] =-(TorsoBend/41);
		joint_state.name[35] ="TSJ13";
		joint_state.position [35] =-(TorsoBend/41);
		joint_state.name[36] ="TSJ14";
		joint_state.position [36] =-(TorsoBend/41);
		joint_state.name[37] ="TSJ15";
		joint_state.position [37] =-(TorsoBend/41);	
		joint_state.name[38] ="TSJ16";
		joint_state.position [38] =-(TorsoBend/41);
		joint_state.name[39] ="TSJ17";
		joint_state.position [39] =-(TorsoBend/41);
		joint_state.name[40] ="TSJ18";
		joint_state.position [40] =-(TorsoBend/41);
		joint_state.name[41] ="TSJ19";
		joint_state.position [41] =-(TorsoBend/41);
		joint_state.name[42] ="TSJ20";
		joint_state.position [42] =-(TorsoBend/41);
		joint_state.name[43] ="TSJ21";
		joint_state.position [43] =-(TorsoBend/41);
		joint_state.name[44] ="TSJ22";
		joint_state.position [44] =-(TorsoBend/41);		
		
		//lower spine
		joint_state.name[45] ="LSJ2";
		joint_state.position [45] =-(TorsoBend/41);
		joint_state.name[46] ="LSJ3";
		joint_state.position [46] =-(TorsoBend/41);
		joint_state.name[47] ="LSJ4";
		joint_state.position [47] =-(TorsoBend/41);
		joint_state.name[48] ="LSJ5";
		joint_state.position [48] =-(TorsoBend/41);
		joint_state.name[49] ="LSJ6";
		joint_state.position [49] =-(TorsoBend/41);
		joint_state.name[50] ="LSJ7";
		joint_state.position [50] =-(TorsoBend/41);
		joint_state.name[51] ="LSJ8";
		joint_state.position [51] =-(TorsoBend/41);
		joint_state.name[52] ="LSJ9";
		joint_state.position [52] =-(TorsoBend/41);	
		joint_state.name[53] ="LSJ10";
		joint_state.position [53] =-(TorsoBend/41);
		joint_state.name[54] ="LSJ11";
		joint_state.position [54] =-(TorsoBend/41);
		joint_state.name[55] ="LSJ12";
		joint_state.position [55] =-(TorsoBend/41);
		joint_state.name[56] ="LSJ13";
		joint_state.position [56] =-(TorsoBend/41);
		joint_state.name[57] ="LSJ14";
		joint_state.position [57] =-(TorsoBend/41);
		joint_state.name[58] ="LSJ15";
		joint_state.position [58] =-(TorsoBend/41);	
		joint_state.name[59] ="LSJ16";
		joint_state.position [59] =-(TorsoBend/41);
		joint_state.name[60] ="LSJ17";
		joint_state.position [60] =-(TorsoBend/41);
		joint_state.name[61] ="LSJ18";
		joint_state.position [61] =-(TorsoBend/41);
		joint_state.name[62] ="LSJ19";
		joint_state.position [62] =-(TorsoBend/41);
		joint_state.name[63] ="LSJ20";
		joint_state.position [63] =-(TorsoBend/41);
		


  map<string, double> joint_positions;
  for (unsigned int i=0; i<state->name.size(); ++i)
    joint_positions.insert(make_pair(state->name[i], state->position[i]));


  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = cos(angle)*2;
  odom_trans.transform.translation.y = sin(angle)*2;
  odom_trans.transform.translation.z = .7;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

  g_joint_pub.publish(joint_state);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  ros::Subscriber joint_sub = n.subscribe("fake_sparky", 10, jointCallback);
  g_joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  //tf::TransformBroadcaster broadcaster;
  //ros::Rate loop_rate(30);
  ros::spin();
}


 
