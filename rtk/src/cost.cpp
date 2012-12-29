/***************************************************************************
 *  src/cost.cpp
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
 *  * Neither the name of rtk nor the names of its
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

#include <rtk/cost.h>

namespace rtk
{

  namespace cost
  {

    double LSQUniform(const _FrameArray & source, const _FrameArray & target)
    {
      return spatial::totalSquaredDistance(source, target);
    }
    
    double LSQWeightedEndEffector(const _FrameArray & source, const _FrameArray &  target, const double & ee_weight)
    {
      double cost = 0.0;
      double ee_distance2 = 0.0;

      cost += spatial::totalSquaredDistance(source, target);

      ee_distance2 += spatial::getDistanceBetween2(source.back().p, target.back().p);
      
      cost+= ee_distance2*ee_weight*(target.size() - 1);
      
      return cost;
    }
    
    double LSQWeightedPairs(const _FrameArray & source, const _FrameArray & target, const _WeightArray & weights)
    {
      double cost = 0.0;
      
      // TODO: Add proper error handling here
      if(weights.size() != source.size() * target.size())
	return 0.0;
      
      _WeightArray::const_iterator weight_it = weights.begin();
      for(_FrameArray::const_iterator target_it = target.begin(); target_it != target.end(); ++target_it)
	{
	  for(_FrameArray::const_iterator source_it = source.begin(); source_it != source.end(); ++source_it, ++weight_it)
	    {
	      cost += spatial::getDistanceBetween2(target_it->p, source_it->p) * (*weight_it);
	    }
	}
      
      return cost;
    }
    
    double LSQWeightedPairsSimilar(const _FrameArray & source, const _FrameArray & target)
    {
      _FrameArray source_straight, target_straight;
      /// One weight for every target -> source joint pair
      _WeightArray weights(source.size() * target.size());
      
      /**
       * Generate weights ~ inversely proportional to distance squared when chains are 
       * completely straight.
       */

      source_straight = spatial::straightenFrameArray(source);
      target_straight = spatial::straightenFrameArray(target);

      _WeightArray::iterator weight_it = weights.begin();
      for(_FrameArray::const_iterator target_it = target_straight.begin(); target_it != target_straight.end(); ++target_it)
	{
	  for(_FrameArray::const_iterator source_it = source_straight.begin(); source_it != source_straight.end(); ++source_it, ++weight_it)
	    {
	      double distance2 = spatial::getDistanceBetween2(target_it->p, source_it->p);
	      
	      /// Using 1/(1 + d) to avoid infinite values when joints are very close
	      *weight_it = 1.0/(1.0 + distance2);
	    }
	}
      
      return LSQWeightedPairs(source, target, weights);
    }
    
  } // cost
} // rtk
