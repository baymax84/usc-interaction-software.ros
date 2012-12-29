#ifndef RTK_RTK_H_
#define RTK_RTK_H_

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace rtk
{
  typedef KDL::Vector   _Vector;
  typedef KDL::Rotation _Rotation;
  typedef KDL::Frame    _Frame;
  typedef KDL::Joint    _Joint;
  typedef KDL::Segment  _Segment;
  typedef KDL::Chain    _Chain;
  typedef KDL::JntArray _JntArray;
  
  typedef std::vector<double> _WeightArray;
  typedef std::vector<_Frame> _FrameArray;
}

#endif // RTK_RTK_H_
