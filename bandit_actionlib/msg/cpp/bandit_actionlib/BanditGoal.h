/* auto-generated by genmsg_cpp from /home/kitty/ros/stacks/interaction-ros-pkg/bandit/bandit_actionlib/msg/BanditGoal.msg.  Do not edit! */
#ifndef BANDIT_ACTIONLIB_BANDITGOAL_H
#define BANDIT_ACTIONLIB_BANDITGOAL_H

#include <string>
#include <vector>
#include "ros/message.h"
#include "ros/debug.h"
#include "ros/assert.h"
#include "ros/time.h"

namespace bandit_actionlib
{

//! \htmlinclude BanditGoal.msg.html

class BanditGoal : public ros::Message
{
public:
  typedef boost::shared_ptr<BanditGoal> Ptr;
  typedef boost::shared_ptr<BanditGoal const> ConstPtr;

  typedef std::string _frame_type;

  std::string frame;

  BanditGoal() : ros::Message()
  {
  }
  BanditGoal(const BanditGoal &copy) : ros::Message(),
    frame(copy.frame)
  {
    (void)copy;
  }
  BanditGoal &operator =(const BanditGoal &copy)
  {
    if (this == &copy)
      return *this;
    frame = copy.frame;
    return *this;
  }
  virtual ~BanditGoal() 
  {
  }
  inline static std::string __s_getDataType() { return std::string("bandit_actionlib/BanditGoal"); }
  inline static std::string __s_getMD5Sum() { return std::string("a054304ca480f0e0c67ec5f261591b09"); }
  inline static std::string __s_getMessageDefinition()
  {
    return std::string(
    "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
    "#goal definition\n"
    "string frame\n"
    "\n"
    "\n"
    );
  }
  inline virtual const std::string __getDataType() const { return __s_getDataType(); }
  inline virtual const std::string __getMD5Sum() const { return __s_getMD5Sum(); }
  inline virtual const std::string __getMessageDefinition() const { return __s_getMessageDefinition(); }
  inline uint32_t serializationLength() const
  {
    unsigned __l = 0;
    __l += 4 + frame.length(); // frame
    return __l;
  }
  virtual uint8_t *serialize(uint8_t *write_ptr,
#if defined(__GNUC__)
                             __attribute__((unused)) uint32_t seq) const
#else
                             uint32_t seq) const
#endif
  {
    unsigned __ros_frame_len = frame.length();
    SROS_SERIALIZE_PRIMITIVE(write_ptr, __ros_frame_len);
    SROS_SERIALIZE_BUFFER(write_ptr, frame.c_str(), __ros_frame_len);
    return write_ptr;
  }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    unsigned __ros_frame_len;
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, __ros_frame_len);
    frame = std::string((const char *)read_ptr, __ros_frame_len);
    read_ptr += __ros_frame_len;
    return read_ptr;
  }
};

typedef boost::shared_ptr<BanditGoal> BanditGoalPtr;
typedef boost::shared_ptr<BanditGoal const> BanditGoalConstPtr;


}

#endif
