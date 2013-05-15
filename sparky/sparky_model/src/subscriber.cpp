// preprocessor directives
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
using namespace std;
using namespace ros;

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

// global variables
ros::Publisher g_joint_pub;

void jointCallback(const sensor_msgs::JointStateConstPtr &state)
{
  geometry_msgs::TransformStamped odom_trans;
  sensor_msgs::JointState         joint_state;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id  = "world";

  double HeadNod, Mouth, EyesLtRt, EyeBlink, LtArmOut,
         LtArmForward, RtArmForward, RtArmOut, LtElbow, RtElbow, LtWrist,
         RtWrist, LtFootForward, RtFootForward, LtFootUp, RtFootUp,
         TorsoBend, HeadTurn;
  double angle = 0.0l;

  float radcorrect = M_PI / 180.0f;
  float L1         = 4.95f; // length of calf (cm)
  float L2         = 4.95f; // length of thigh (cm)

  // transform vertical and horizontal inputs into angles for Left Leg
  float h_left  = state->position[16];
  
  if    (h_left == 0.0f) h_left = 0.0f;
  else                   h_left = (h_left - 3.5f)*1.8457f/4.754f; // set scaling factor
   
   //h_left  = h_left*1.94118f/5.0f;  
  float w_left  = L1 + L2 - h_left - .387252f;
  float dx_left = state->position[17];
  
  //set scaling factor
  if    (dx_left >= 0.0f) dx_left = dx_left*1.94118f/5.0f;
  else                    dx_left = dx_left*1.55292f/4.0f;
  
  float r2_left = pow(w_left, 2) + pow(dx_left, 2);

  // theta2_left is the angle formed by bending the left knee
  float theta2_left =
    acos((r2_left - pow(L1, 2) - pow(L2, 2)) / (-1.0f * 2.0f * L1 * L2));
  float vert_left = sin(theta2_left);

  // theta1_left is the angle formed by the left thigh
  // with respect to the vertical
  float theta1_left =
    atan(dx_left / w_left) + asin(L2 * vert_left / pow(r2_left, 0.5f));

  // transform vertical and horrizontal inputs into angles for Right Leg
  float h_right  = state->position[14];
  
    if (h_right == 0.0f) h_right = 0.0f;
  else                   h_right = (h_right - 3.25f)*1.8457f/4.754f; // set scaling factor
  
  float w_right  = L1 + L2 - h_right - .387252f; // (10.0f - h_right) / 30.0f;
  float dx_right = state->position[15];
    
  //set scaling factor
  if    (dx_right >= 0.0f) dx_right = dx_right*1.94118f/5.0f;
  else                     dx_right = dx_right*1.55292f/4.0f;
  
  float r2_right = pow(w_right, 2) + pow(dx_right, 2);

  // theta2_right is the angle formed by bending the left knee
  float theta2_right =
    acos((r2_right - pow(L1, 2) - pow(L2, 2)) / (-1.0f * 2.0f * L1 * L2));
  float vert_right   = sin(theta2_right);

  // theta1_right is the angle formed by the right thigh
  // with respect to the vertical
  float theta1_right =
    atan(dx_right / w_right) + asin(L2 * vert_right / pow(r2_right, 0.5f));

  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(64);
  joint_state.position.resize(64);
        
  ROS_INFO("running ok");
             
  // head
  joint_state.name[0]     = "Mouth";
  Mouth                   = radcorrect * state->position[0];
  joint_state.position[0] = Mouth;
  joint_state.name[1]     = "HeadNod";
  HeadNod                 = radcorrect * state->position[1];
  joint_state.position[1] = HeadNod;
  joint_state.name[2]    = "HeadTurn";
  HeadTurn                = radcorrect * state->position[2];
  joint_state.position[2] = HeadTurn;

  // eyes
  joint_state.name[3]      = "EyesLtRt";
  EyesLtRt                 = radcorrect * state->position[13];
  joint_state.position[3]  = EyesLtRt;
  joint_state.name[4]      = "LeyeJoint";
  joint_state.position[4]  = EyesLtRt;        
  joint_state.name[5]      = "EyeBlink";
  EyeBlink                 = -radcorrect * state->position[12];
  joint_state.position[5]  = -EyeBlink;
  joint_state.name[6]      = "RLidJoint";
  joint_state.position[6]  = -EyeBlink;
  joint_state.name[7]      = "LPinJoint";
  joint_state.position[7]  = -1.0l * EyesLtRt;
  joint_state.name[8]      = "EyeBeamJoint";
  joint_state.position[8]  = 0.5113l * EyesLtRt;
  joint_state.name[9]      = "EyeBallPinJoint";
  joint_state.position[9]  = EyesLtRt;
  joint_state.name[10]     = "EyeBallPinJoint2";
  joint_state.position[10] = EyesLtRt;

  // left arm
  joint_state.name[11]     = "LtArmOut";
  LtArmOut                 = radcorrect * state->position[7];
  joint_state.position[11] = LtArmOut;
  joint_state.name[12]     = "LtArmForward";
  LtArmForward             = radcorrect * state->position[6];
  joint_state.position[12] = LtArmForward;
  joint_state.name[13]     = "LtElbow";
  LtElbow                  = radcorrect * state->position[8];
  joint_state.position[13] = -LtElbow;
  joint_state.name[14]     = "LtWrist";
  LtWrist                  = radcorrect * state->position[10];
  joint_state.position[14] = LtWrist;

  // right arm
  joint_state.name[15]     = "RtArmOut";
  RtArmOut                 = radcorrect * state->position[4];
  joint_state.position[15] = -RtArmOut;       
  joint_state.name[16]     = "RtArmForward";
  RtArmForward             = radcorrect * state->position[3];
  joint_state.position[16] = RtArmForward;
  joint_state.name[17]     = "RtElbow";
  RtElbow                  = radcorrect * state->position[5];
  joint_state.position[17] = RtElbow;
  joint_state.name[18]     = "RtWrist";
  RtWrist                  = radcorrect * state->position[9];
  joint_state.position[18] = -RtWrist;

  // left leg
  joint_state.name[19]     = "LtFootForward";
  LtFootForward            = theta1_left;
  joint_state.position[19] = LtFootForward;
  joint_state.name[20]     = "LtFootUp";
  LtFootUp                 = theta2_left + M_PI;
  joint_state.position[20] = LtFootUp;

  // right leg
  joint_state.name[21]     = "RtFootForward";
  RtFootForward            = theta1_right;
  joint_state.position[21] = RtFootForward;
  joint_state.name[22]     = "RtFootUp";
  RtFootUp                 = theta2_right + M_PI;
  joint_state.position[22] = RtFootUp;

  // spine
  TorsoBend                  = radcorrect * state->position[11];
  double torso_bend_interval = TorsoBend / 41.0l;

  // upper spine
  joint_state.name[23]       = "TorsoBend";
  joint_state.position[23] = torso_bend_interval;
  joint_state.name[24]     = "TSJ2";
  joint_state.position[24] = torso_bend_interval;
  joint_state.name[25]     = "TSJ3";
  joint_state.position[25] = torso_bend_interval;
  joint_state.name[26]     = "TSJ4";
  joint_state.position[26] = torso_bend_interval;
  joint_state.name[27]     = "TSJ5";
  joint_state.position[27] = torso_bend_interval;	
  joint_state.name[28]     = "TSJ6";
  joint_state.position[28] = torso_bend_interval;
  joint_state.name[29]     = "TSJ7";
  joint_state.position[29] = torso_bend_interval;
  joint_state.name[30]     = "TSJ8";
  joint_state.position[30] = torso_bend_interval;
  joint_state.name[31]     = "TSJ9";
  joint_state.position[31] = torso_bend_interval;
  joint_state.name[32]     = "TSJ10";
  joint_state.position[32] = torso_bend_interval;
  joint_state.name[33]     = "TSJ11";
  joint_state.position[33] = torso_bend_interval;
  joint_state.name[34]     = "TSJ12";
  joint_state.position[34] = torso_bend_interval;
  joint_state.name[35]     = "TSJ13";
  joint_state.position[35] = torso_bend_interval;
  joint_state.name[36]     = "TSJ14";
  joint_state.position[36] = torso_bend_interval;
  joint_state.name[37]     = "TSJ15";
  joint_state.position[37] = torso_bend_interval;	
  joint_state.name[38]     = "TSJ16";
  joint_state.position[38] = torso_bend_interval;
  joint_state.name[39]     = "TSJ17";
  joint_state.position[39] = torso_bend_interval;
  joint_state.name[40]     = "TSJ18";
  joint_state.position[40] = torso_bend_interval;
  joint_state.name[41]     = "TSJ19";
  joint_state.position[41] = torso_bend_interval;
  joint_state.name[42]     = "TSJ20";
  joint_state.position[42] = torso_bend_interval;
  joint_state.name[43]     = "TSJ21";
  joint_state.position[43] = torso_bend_interval;
  joint_state.name[44]     = "TSJ22";
  joint_state.position[44] = torso_bend_interval;

  // lower spine
  joint_state.name[45]     = "LSJ2";
  joint_state.position[45] = torso_bend_interval;
  joint_state.name[46]     = "LSJ3";
  joint_state.position[46] = torso_bend_interval;
  joint_state.name[47]     = "LSJ4";
  joint_state.position[47] = torso_bend_interval;
  joint_state.name[48]     = "LSJ5";
  joint_state.position[48] = torso_bend_interval;
  joint_state.name[49]     = "LSJ6";
  joint_state.position[49] = torso_bend_interval;
  joint_state.name[50]     = "LSJ7";
  joint_state.position[50] = torso_bend_interval;
  joint_state.name[51]     = "LSJ8";
  joint_state.position[51] = torso_bend_interval;
  joint_state.name[52]     = "LSJ9";
  joint_state.position[52] = torso_bend_interval;	
  joint_state.name[53]     = "LSJ10";
  joint_state.position[53] = torso_bend_interval;
  joint_state.name[54]     = "LSJ11";
  joint_state.position[54] = torso_bend_interval;
  joint_state.name[55]     = "LSJ12";
  joint_state.position[55] = torso_bend_interval;
  joint_state.name[56]     = "LSJ13";
  joint_state.position[56] = torso_bend_interval;
  joint_state.name[57]     = "LSJ14";
  joint_state.position[57] = torso_bend_interval;
  joint_state.name[58]     = "LSJ15";
  joint_state.position[58] = torso_bend_interval;	
  joint_state.name[59]     = "LSJ16";
  joint_state.position[59] = torso_bend_interval;
  joint_state.name[60]     = "LSJ17";
  joint_state.position[60] = torso_bend_interval;
  joint_state.name[61]     = "LSJ18";
  joint_state.position[61] = torso_bend_interval;
  joint_state.name[62]     = "LSJ19";
  joint_state.position[62] = torso_bend_interval;
  joint_state.name[63]     = "LSJ20";
  joint_state.position[63] = torso_bend_interval;

  map<string, double> joint_positions;
  for (unsigned int i = 0, n = state->name.size(); i < n; ++i)
    joint_positions.insert(make_pair(state->name[i], state->position[i]));

  odom_trans.header.stamp            = ros::Time::now();
  odom_trans.transform.translation.x = cos(angle) * 2.0l;
  odom_trans.transform.translation.y = sin(angle) * 2.0l;
  odom_trans.transform.translation.z = 0.7l;
  odom_trans.transform.rotation      =
    tf::createQuaternionMsgFromYaw(angle + M_PI / 2.0l);

  g_joint_pub.publish(joint_state);
} // jointCallback(const sensor_msgs::JointStateConstPtr &)

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  ros::Subscriber joint_sub = n.subscribe("fake_sparky", 10, jointCallback);
  g_joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  //tf::TransformBroadcaster broadcaster;
  //ros::Rate loop_rate(30);
  ros::spin();
} // main(int, char**)

