/***************************************************************************
 *  include/rtk/cost.h
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

#ifndef RTK_COST_H_
#define RTK_COST_H_

#include <math.h>

#include <vector>

#include <rtk/rtk.h>
#include <rtk/math_impl.h>

namespace rtk
{
  namespace cost
  {
    
    /** 
     * Compute distance between all joint pairs in a least squared sense;
     * 
     * @param source Source chain
     * @param target Target chain
     * 
     * @return Cost
     */
    double LSQUniform(const _FrameArray & source, const _FrameArray & target);
    
    /** 
     * Compute distance between all joint pairs 
     * in a least-squared sense, then add weighted distance between end effectors.
     *
     * @param source Source chain
     * @param target Target chain
     * @param ee_weight End effector weight
     * 
     * @return Cost
     */
    double LSQWeightedEndEffector(const _FrameArray & source, const _FrameArray & target, const double &  ee_weight);

    /** 
     * Compute distance between all joint pairs 
     * in a least-squared sense, with  the distance of 
     * each pair multiplied by a given weight.
     * 
     * @param source Source chain
     * @param target Target chain
     * @param weights Vector of weights for each pair
     * 
     * @return Cost
     */
    double LSQWeightedPairs(const _FrameArray &  source, const _FrameArray & target, const _WeightArray & weights);
      
    /** 
     * Weight joint pairs based on their distance when both chains 
     * are completely straigt.
     *
     * @param source Source chain
     * @param target Target chain
     * 
     * @return Cost 
     */
    double LSQWeightedPairsSimilar(const _FrameArray & source, const _FrameArray & target);
      
    
  } // cost
} // rtk

#endif // RTK_COST_H_
