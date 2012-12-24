/***************************************************************************
 *  src/math_impl.cpp
 *  --------------------
 *
 *  Copyright (c) 2012, Dylan J. Foster
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of krt nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#include <krt/math_impl.h>

namespace krt
{

  namespace spatial
  {

    _Vector getDistance(const _Vector & vec1, const _Vector & vec2)
    {
      return vec2 - vec1;
    }
  
  
    double getDistanceBetween(const _Vector & vec1, const _Vector & vec2)
    {
      return getDistance(vec1, vec2).Norm();
    }
  
  
    double getDistanceBetween2(const _Vector & vec1, const _Vector & vec2)
    {
      return pow(getDistanceBetween(vec1, vec2), 2);
    }
  

    double getLength(const _Chain & chain)
    {
      const unsigned int nr_segments = chain.getNrOfSegments();
      double length = 0.0;
    
      for(unsigned int i = 0; i < nr_segments; ++i)
	{
	  length += chain.getSegment(i).getFrameToTip().p.Norm();
	}
    
      return length;
    }

  
    _Chain operator * (const _Chain& lhs, const double & rhs)
    {
      _Chain tmp;
    
      const unsigned int nr_segments = lhs.getNrOfSegments();
    
      for(unsigned int i = 0; i < nr_segments; ++i)
	{
	  _Segment segment = tmp.getSegment(i);
	  _Frame frame_scaled = segment.getFrameToTip();
	  frame_scaled.p = frame_scaled.p * rhs;
	    
	  tmp.addSegment( _Segment(segment.getJoint(), 
				   frame_scaled, 
				   segment.getInertia()) );
	}
      return tmp;
    }


    _Chain normalize(const _Chain& chain)
    {
      const double length = getLength(chain);
      return chain * (1/length);
    }
    
    
    double getLength(const _FrameArray & chain)
    {
      double length = 0.0;
      for(auto chain_it = ++chain.begin(); chain_it != chain.end(); ++chain_it)
	{
	  length += getDistanceBetween(chain_it->p, (chain_it-1)->p);
	}
      return length;
    }
    
  
    _FrameArray operator * (const _FrameArray & lhs, const double & rhs)
    {
      _FrameArray tmp = lhs;
	
      for(auto chain_it = tmp.begin(); chain_it != tmp.end(); ++chain_it)
	{
	  chain_it->p = chain_it->p * rhs;
	}
	
      return tmp;
    }

  
    _FrameArray normalize(const _FrameArray & chain)
    {
      const double length = getLength(chain);
      return chain * (1/length);
    }
    
  
    void transform(_FrameArray & chain, const _Frame & transform_frame)
    {
      for(auto chain_it = chain.begin(); chain_it != chain.end(); ++chain_it)
	{
	  _Frame & element = *chain_it;
	  element = transform_frame * element;
	}
    }

  
    void translateToOrigin(_FrameArray & chain)
    {
      _Vector offset = chain.front().p;
      
      for(auto chain_it = chain.begin(); chain_it != chain.end(); ++chain_it)
	{
	  chain_it->p = chain_it->p - offset;
	}
    }
    
  
    int forwardKinematics(const _Chain & chain, const _JntArray & angles, _FrameArray & solved_pose)
    {
      unsigned int nr_joints = chain.getNrOfJoints();
      
      _Frame joint_frame;
      
      // Is it more expensive to allocate this with default frames, or to leave it empty and force it to resize when I push back frames during the next loop?
      solved_pose = _FrameArray(nr_joints);
      
      KDL::ChainFkSolverPos_recursive fksolver(chain);
      
      for(unsigned int i = 0; i < nr_joints; ++i)
	{
	  int error = fksolver.JntToCart(angles, joint_frame, i);
	  if(error) return error;

	  solved_pose[i] = joint_frame;	  
	}
      
      return 0;
    }
    
  
    double totalSquaredDistance(const _FrameArray & pose1, const _FrameArray & pose2)
    {
      double distance = 0.0;

      for(auto pose1_it = pose1.begin(); pose1_it != pose1.end(); ++pose1_it)
	{
	  for(auto pose2_it = pose2.begin(); pose2_it != pose2.end(); ++pose2_it)
	    {
	      distance += getDistanceBetween2(pose1_it->p, pose2_it->p);
	    }
	}
      
      return distance;
    }

  
    _Frame getCentroid(const _FrameArray & frames)
    {
      unsigned int nr_frames = frames.size();
      // Zero vector
      _Vector centroid = KDL::Vector::Zero();
      
      
      for(auto frame_it = frames.begin(); frame_it != frames.end(); ++frame_it)
	{
	  centroid += frame_it->p;
	}
      
      centroid = centroid * (1.0/double(nr_frames));
      
      return _Frame(centroid);
    }
  }

}
