
#include <math.h>
#include <cstdio>
#include "bandit/bandit.h"

using namespace bandit;

#define DTOR( a ) ( ( a ) * M_PI / 180.0f )

//#define DEBUG
//#define HIDE_ERRORS

#define BANDIT_EXCEPT(except, msg, ...) \
  { \
    char buf[256]; \
    snprintf(buf, 256, "%s::%s: " msg, __CLASS__, __FUNCTION__,##__VA_ARGS__); \
    throw except(buf); \
  }

#define __CLASS__ "bandit::Bandit"

Bandit::Bandit() : master_(7)
{
  addJoint(0, "head pitch","bandit_head_tilt_joint",          6, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(-20.0f), DTOR( 30.0f));
  addJoint(1, "head pan", "bandit_head_pan_joint",            2, smartservo::JOINT_B, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(-90.0f), DTOR( 90.0f));
  addJoint(2, "left shoulder F/B","left_torso_shoulder_mounting_joint",   5, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(-30.0f), DTOR(180.0f));
  addJoint(3, "left shoulder I/O","left_shoulder_mounting_shoulder_joint",   4, smartservo::JOINT_B, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(  0.0f), DTOR(150.0f));
  addJoint(4, "left elbow twist","left_shoulder_bicep_joint",    4, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(-90.0f), DTOR( 90.0f));
  addJoint(5, "left elbow","left_bicep_forearm_joint",          3, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(  0.0f), DTOR(110.0f));
  addJoint(6, "left wrist twist","left_forearm_wrist_joint",    3, smartservo::JOINT_B, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(-90.0f), DTOR( 90.0f));
  addJoint(7, "left wrist tilt","left_wrist_hand_joint",     3, smartservo::JOINT_B, smartservo::V1_SERVO, 1, 1/255.0, 0.0, -0.5, 0.5);
  addJoint(8, "left hand grab","left_hand_thumb_joint",      3, smartservo::JOINT_A, smartservo::V1_SERVO, 1, 1/255.0, 0.0, -0.5, 0.0);
  addJoint(9, "right shoulder F/B","right_torso_shoulder_mounting_joint",  2, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(-30.0f), DTOR(180.0f));
  addJoint(10, "right shoulder I/O","right_shoulder_mounting_shoulder_joint", 1, smartservo::JOINT_B, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(  0.0f), DTOR(150.0f));
  addJoint(11, "right elbow twist","right_shoulder_bicep_joint",  1, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(-90.0f), DTOR( 90.0f));
  addJoint(12, "right elbow","right_bicep_forearm_joint", 0, smartservo::JOINT_A, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(  0.0f), DTOR(110.0f));
  addJoint(13, "right wrist twist","right_forearm_wrist_joint",  0, smartservo::JOINT_B, smartservo::SMART_SERVO, 1, 2*M_PI/4096.0, M_PI, DTOR(-90.0f), DTOR( 90.0f));
  addJoint(14, "right wrist tilt", "right_wrist_hand_joint",  0, smartservo::JOINT_B, smartservo::V1_SERVO, 1, 1/255.0, 0.0, -0.5, 0.5);
  addJoint(15, "right hand grab","right_hand_thumb_joint",    0, smartservo::JOINT_A, smartservo::V1_SERVO, 1, 1/255.0, 0.0, -0.5, 0.0);
  addJoint(16, "eyebrows", "eyebrows",          5, smartservo::JOINT_B, smartservo::HOBBY_SERVO, 1, 1/255.0, 0.0, -0.1, 0.4);
  addJoint(17, "mouth top","mouth_top",        6, smartservo::JOINT_A, smartservo::HOBBY_SERVO, 1, 1/255.0, 0.0, -0.25, 0.25);
  addJoint(18, "mouth bottom","mouth_bottom",       6, smartservo::JOINT_B, smartservo::HOBBY_SERVO, 1, 1/255.0, 0.0, -0.25, 0.25);
}
    
Bandit::~Bandit()
{

}

void Bandit::addJoint(int id, std::string name, std::string rname, int mod_id, smartservo::WhichJoint which, smartservo::JointType type,
                      int8_t direction, double scale, double offset, double min, double max)
{
  Joint joint;
  joint.mod_id = mod_id;
  joint.name = name;
  joint.ros_name = rname;
  joint.which = which;
  joint.type = type;
  joint.direction = direction;
  joint.scale = scale;
  joint.offset = offset;
  joint.min = min;
  joint.max = max;

  joints_[id] = joint;

  master_.setJointType(mod_id, type);
  
}

void Bandit::registerStateCB(boost::function<void()> callback)
{
  master_.registerStateCB(callback);
}
    
void Bandit::setJointPIDConfig(int16_t id,
                               int16_t p, int16_t i, int16_t d,
                               int16_t i_min, int16_t i_max,
                               uint8_t e_min, int16_t offset)

{
  std::map<uint16_t, Joint>::iterator joint = joints_.find(id);

  if (joint == joints_.end())
    BANDIT_EXCEPT(BanditException, "No joint with id %d", id);

  if (joint->second.type != smartservo::SMART_SERVO)
    BANDIT_EXCEPT(BanditException, "Tried to set PID gains on: %s, which is not a SMART_SERVO\n", joint->second.name.c_str());

  master_.setJointPIDConfig(joint->second.mod_id, joint->second.which,
                            p, i, d,
                            i_min, i_max,
                            e_min, offset);
}


void Bandit::setJointDirection(uint16_t id, int8_t direction)
{
  std::map<uint16_t, Joint>::iterator joint = joints_.find(id);

  if (joint == joints_.end())
    BANDIT_EXCEPT(BanditException, "No joint with id %d", id);

  if ((direction != -1) && (direction != 1))
  { // if you remove these {}'s, this will not compile!?!
    BANDIT_EXCEPT(BanditException, "Tried to set joint direction with invalid value");
  }
  else
    joint->second.direction = direction;
}


void Bandit::setJointOffset(uint16_t id, double offset)
{
  std::map<uint16_t, Joint>::iterator joint = joints_.find(id);

  if (joint == joints_.end())
    BANDIT_EXCEPT(BanditException, "No joint with id %d", id);

  if (joint->second.type == smartservo::SMART_SERVO)
    joint->second.offset = offset + joint->second.direction * M_PI;
  else
    joint->second.offset = offset;
}


void Bandit::setJointPos(uint16_t id, double angle)
{
  std::map<uint16_t, Joint>::iterator joint = joints_.find(id);

  if (joint == joints_.end())
    BANDIT_EXCEPT(BanditException, "No joint with id %d", id);
/*
  if (angle > joint->second.max)
    angle = joint->second.max;
  
  if (angle < joint->second.min)
    angle = joint->second.min;
*/
  int16_t pos = (joint->second.direction * angle + joint->second.offset) / joint->second.scale;
  
  switch (joint->second.type)
  {
  case smartservo::SMART_SERVO:
    master_.setJointPos(joint->second.mod_id, joint->second.which, pos);
    break;
  case smartservo::HOBBY_SERVO:
  case smartservo::V1_SERVO:
    if (pos > 255)
      pos = 255;
    if (pos < 0)
      pos = 0;
    master_.setJointServoPos(joint->second.mod_id, joint->second.which, pos);
    break;
  default:
    BANDIT_EXCEPT(BanditException, "Tried to set a joint position for a joint with unspecified type");
  }
}

double Bandit::getJointPos(uint16_t id)
{
  std::map<uint16_t, Joint>::iterator joint = joints_.find(id);

  if (joint == joints_.end())
    BANDIT_EXCEPT(BanditException, "No joint with id %d", id);

  int16_t pos = 0;

  switch (joint->second.type)
  {
  case smartservo::SMART_SERVO:
    pos = master_.getJointPos(joint->second.mod_id, joint->second.which);
    break;
  case smartservo::HOBBY_SERVO:
  case smartservo::V1_SERVO:
    pos = master_.getJointServoPos(joint->second.mod_id, joint->second.which);
    break;
  default:
    BANDIT_EXCEPT(BanditException, "Tried to get a joint position for a joint with unspecified type");
  }

  return joint->second.direction * (joint->second.scale * (double)(pos) - joint->second.offset);
}


smartservo::JointType Bandit::getJointType(int id)
{
  std::map<uint16_t, Joint>::iterator joint = joints_.find(id);

  if (joint == joints_.end())
    BANDIT_EXCEPT(BanditException, "No joint with id %d", id);

  return joint->second.type;
}

std::string Bandit::getJointName(int id)
{
  std::map<uint16_t, Joint>::iterator joint = joints_.find(id);

  if (joint == joints_.end())
    BANDIT_EXCEPT(BanditException, "No joint with id %d", id);

  return joint->second.name;
}

std::string Bandit::getJointRosName(int id)
{
  std::map<uint16_t, Joint>::iterator joint = joints_.find(id);

  if (joint == joints_.end())
    BANDIT_EXCEPT(BanditException, "No joint with id %d", id);

  return joint->second.ros_name;
}

double Bandit::getJointMin(int id)
{
  std::map<uint16_t, Joint>::iterator joint = joints_.find(id);

  if (joint == joints_.end())
    BANDIT_EXCEPT(BanditException, "No joint with id %d", id);

  return joint->second.min;
}

double Bandit::getJointMax(int id)
{
  std::map<uint16_t, Joint>::iterator joint = joints_.find(id);

  if (joint == joints_.end())
    BANDIT_EXCEPT(BanditException, "No joint with id %d", id);

  return joint->second.max;
}
