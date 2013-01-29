/***************************************************************************
 *  src/pose_solver_lsq.cpp
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

#include <rtk/pose_solver_lsq.h>

namespace rtk
{

  PoseSolverLsq::PoseSolverLsq(_Chain const & target_chain) :
    target_chain_(target_chain),
    solved_(false),
    fmin_type_(gsl_multimin_fminimizer_nmsimplex2)
  {
    nr_angles_ = target_chain_.getNrOfJoints();
    target_angles_ = _JntArray(nr_angles_);
  }

  PoseSolverLsq::~PoseSolverLsq(){}

  int PoseSolverLsq::update(const _FrameArray & source_pose, double const & ee_weight, 
			    double const & epsilon, 
			    unsigned int const & max_iterations)
  {
    solved_ = false;

    // TODO: Don't let argument leave scope of this function
    source_pose_ = source_pose;
    ee_weight_ = ee_weight;

    // TODO: Take these values as a parameter
    // const unsigned int max_iterations = 1000;
    // const double step_threshold = 0.0001;
    
    // GSL multimin variables
    gsl_multimin_fminimizer *minimizer;      
    gsl_vector *init_guess, *init_step;
    gsl_multimin_function minex_func;

    unsigned int iteration = 0;
    double step_size;
    int minimize_status;
    
    // TODO: change to a more intelligent guess (IK from source?)
    init_guess = gsl_vector_alloc(nr_angles_);
    gsl_vector_set_all(init_guess, 0.0);
      
    // Using arbitrary value for initial step size
    init_step = gsl_vector_alloc(nr_angles_);
    gsl_vector_set_all(init_step, M_PI/10.0);
      
    minex_func.n = nr_angles_;
    
    // TODO: Figure out how to lose the cost helper fucntion
    minex_func.f = &costHelper;
    minex_func.params = this;
        
    minimizer = gsl_multimin_fminimizer_alloc(fmin_type_, nr_angles_);
    gsl_multimin_fminimizer_set(minimizer, &minex_func, init_guess, init_step);
    
    do 
      {
	iteration++;
	minimize_status = gsl_multimin_fminimizer_iterate(minimizer);
	
	if(minimize_status)
	  {
	    // ROS_WARN("Pose solver failed");
	    break;
	  }
	step_size = gsl_multimin_fminimizer_size(minimizer);
	
	// if(minimizer->fval < epsilon)
	//   minimize_status = GSL_SUCCESS;
	
	minimize_status = gsl_multimin_test_size(step_size, epsilon);
	// printf("cost is %f\n", minimizer->fval);

	// minimize_status = gsl_multimin_test_gradient(minimizer->gradient, epsilon);
	
	// if(minimize_status == GSL_SUCCESS)
	// printf("Pose solver converged to minimum at:\n");
	
	  std::stringstream angle_string;
	for(unsigned int i = 0; i < nr_angles_; i++)
	  {
	    angle_string << i+1 << ". " << gsl_vector_get(minimizer->x, i) << '\t';
	  }
	
	// printf("Iteration: %u, error: %.4f, step: %.4f\n", iteration, minimizer->fval, step_size);
	// printf("Angles: %s\n", angle_string.str().c_str());
		 
      }
    while(minimize_status == GSL_CONTINUE && iteration < max_iterations);
	
    gsl_vector_free(init_guess);
    gsl_vector_free(init_step);
    gsl_multimin_fminimizer_free(minimizer);
    
    if(minimize_status == GSL_SUCCESS) 
      solved_ = true;
    else
      solved_ = false;
    
    return solved_;
  }

  _FrameArray PoseSolverLsq::getTargetPose() const
  {
    _FrameArray target_pose;

    // perform forward kinematics using current value of target angles
    spatial::forwardKinematics(target_chain_, target_angles_, target_pose);
        
    return target_pose;
  }

  _JntArray PoseSolverLsq::getTargetAngles() const
  {
    return target_angles_;
  }
  
  double PoseSolverLsq::cost(const gsl_vector * angles)
  {
    // TODO: get ee_cost from a config or weight function
    // const double ee_weight = 5.0;
    // double ee_distance2 = 0.0;
    
    // double cost = 0.0;
    
    _FrameArray target_pose;

    for(unsigned int i = 0; i < nr_angles_; ++i)
      {
	target_angles_(i) = gsl_vector_get(angles, i);
      }

    // Perform fk and find total distance
    spatial::forwardKinematics(target_chain_, target_angles_, target_pose);

    // Translate source chain such that it starts at origin 
    // (target is assumed to start at origin already)
    
    // Normalize chains to unit length
    target_pose = spatial::normalize(target_pose);
    source_pose_ = spatial::normalize(source_pose_);
    // printf("target length %f, source length %f\n",spatial::getLength(target_pose), spatial::getLength(source_pose_));
    
    // cost += spatial::totalSquaredDistance(target_pose, source_pose_);
    
    // // Add cost from end effector weight
    // ee_distance2 += spatial::getDistanceBetween2(target_pose.back().p, 
    // 						 source_pose_.back().p);

    // // TODO: this should really be nr_joints to make conceptual sense, but the value of nr_angles is the same
    // cost += ee_distance2*ee_weight_*(nr_angles_-1);
   
    // return cost;

    // return cost::LSQWeightedPairsSimilar(source_pose_, target_pose);
    return cost::LSQWeightedEndEffector(source_pose_, target_pose, 10.0);
  }
  

}

