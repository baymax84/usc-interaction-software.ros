#ifndef KRT_KRT_H_
#define KRT_KRT_H_

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace krt
{
  typedef KDL::Vector   _Vector;
  typedef KDL::Rotation _Rotation;
  typedef KDL::Frame    _Frame;
  typedef KDL::Joint    _Joint;
  typedef KDL::Segment  _Segment;
  typedef KDL::Chain    _Chain;
  typedef KDL::JntArray _JntArray;
}

#endif // KRT_KRT_H_
