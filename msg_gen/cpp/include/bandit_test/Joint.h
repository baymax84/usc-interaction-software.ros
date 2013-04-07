/* Auto-generated by genmsg_cpp for file /home/prashanth/groovy_workspace/sandbox/bandit_test/msg/Joint.msg */
#ifndef BANDIT_TEST_MESSAGE_JOINT_H
#define BANDIT_TEST_MESSAGE_JOINT_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace bandit_test
{
template <class ContainerAllocator>
struct Joint_ {
  typedef Joint_<ContainerAllocator> Type;

  Joint_()
  : id(0)
  , angle(0.0)
  {
  }

  Joint_(const ContainerAllocator& _alloc)
  : id(0)
  , angle(0.0)
  {
  }

  typedef uint32_t _id_type;
  uint32_t id;

  typedef double _angle_type;
  double angle;


  typedef boost::shared_ptr< ::bandit_test::Joint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bandit_test::Joint_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Joint
typedef  ::bandit_test::Joint_<std::allocator<void> > Joint;

typedef boost::shared_ptr< ::bandit_test::Joint> JointPtr;
typedef boost::shared_ptr< ::bandit_test::Joint const> JointConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::bandit_test::Joint_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::bandit_test::Joint_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace bandit_test

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::bandit_test::Joint_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::bandit_test::Joint_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::bandit_test::Joint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cdd826a1ef97f557d580d6aa396f4f5a";
  }

  static const char* value(const  ::bandit_test::Joint_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xcdd826a1ef97f557ULL;
  static const uint64_t static_value2 = 0xd580d6aa396f4f5aULL;
};

template<class ContainerAllocator>
struct DataType< ::bandit_test::Joint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bandit_test/Joint";
  }

  static const char* value(const  ::bandit_test::Joint_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::bandit_test::Joint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint32 id\n\
float64 angle\n\
";
  }

  static const char* value(const  ::bandit_test::Joint_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::bandit_test::Joint_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::bandit_test::Joint_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.id);
    stream.next(m.angle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Joint_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bandit_test::Joint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::bandit_test::Joint_<ContainerAllocator> & v) 
  {
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "angle: ";
    Printer<double>::stream(s, indent + "  ", v.angle);
  }
};


} // namespace message_operations
} // namespace ros

#endif // BANDIT_TEST_MESSAGE_JOINT_H

