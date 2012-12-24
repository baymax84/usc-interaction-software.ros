/***************************************************************************
 *  include/rtk/pose_solver_lsq.h
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

#ifndef RTK_POSESOLVERLSQ_H_
#define RTK_POSESOLVERLSQ_H_

// gsl multivariable minimization library
#include <gsl/gsl_multimin.h>

// kdl
#include <kdl/chainfksolverpos_recursive.hpp>

#include <rtk/math_impl.h>

namespace rtk
{
  
  namespace
    {
      /** Helper function compute cost for minimization for a given angle configuration. 
     * This is function exists as a hack to work around gsl's C syntax
     *
     * @param angles Joint angles for target chain
     * @param params A pointer to an instance of this class 
     * 
     * @return total distance between chains for a given angle
     */
      double costHelper(const gsl_vector * angles, void * params);
    }
  
  /** 
   * @brief This class represents an algorithm to retarget a target kinematic chain to match 
   * a source kinematic chain as closely as possible in a least-squared error sense.
   * 
   */
  class PoseSolverLsq
  {
    /// member variable declarations
  private:
    _Chain const target_chain_;
    _JntArray target_angles_;
    unsigned int nr_angles_;
    
    // TODO: Start using this variable to throw warnings for getter functions
    bool solved_;
        
    // TODO: Find a way to not need these declarations
    _FrameArray source_pose_;
    double ee_weight_;
    
    // GSL multivariable minimization
    // Nelder-Mead Simplex algorithm
    const gsl_multimin_fminimizer_type *fmin_type_;

    /// member function declarations
  public:
    friend double costHelper(const gsl_vector * angles, void * params);
    
    /** 
     * Default constructor
     * 
     * @param target_chain The chain that this pose solver instance will be retargeting 
     */
    PoseSolverLsq(_Chain const & target_chain);
    
    /** 
     * Default destructor
     * 
     */
    ~PoseSolverLsq() = default;

    /** 
     * Retarget the target chain using given source a set of positions 
     * composing a source chain.
     * 
     * @param source_pose Target chain will be retargeted to match this pose
     * @param ee_weight Additional weight placed on end effector during minimization
     * @param epsilon Minimization will stop if simplex search step size drops below this value
     * @param max_iterations Minimization will stop if this number of iterations is exceeded
     * @return  True if retargeting was sucessful, false otherwise
     */
    int update(const _FrameArray & source_pose, double const & ee_weight = 5.0, 
	       double const & epsilon = .1, 
	       unsigned int const & max_iterations = 500);
    
    /** 
     * Get the target chain's pose using current joint angles (not guaranteed to be solved)
     * 
     * 
     * @return Target chain pose
     */
    _FrameArray getTargetPose() const;
    
    /** 
     * Get the target chain's joint angles (all zero if pose hasn't been solved)
     * 
     * 
     * @return Joint angles of target chain
     */
    _JntArray getTargetAngles() const;
    
    // TODO: fix this so that cost function is hidden
  public:
    
    /** Compute cost used by minimization algorithm for a given angle configuration. 
     *
     * @param angles Joint angles for target chain
     * 
     * @return total squared distance between chains for a given angle
     */
    double cost(const gsl_vector * angles);
  };

  namespace
    {
      double costHelper(const gsl_vector * angles, void * params)
      {
	PoseSolverLsq * caller = (PoseSolverLsq *) params;
	
	return caller->cost(angles);
      }
    }
  
  
} // rtk

#endif // RTK_POSESOLVERLSQ_H_
