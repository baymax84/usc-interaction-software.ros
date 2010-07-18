//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#include <angles/angles.h>
#include <bandit_ik/bandit_arm_ik.h>

/**** List of angles (for reference) *******
      th1 = shoulder/turret pan
      th2 = shoulder/turret lift/pitch
      th3 = shoulder/turret roll
      th4 = elbow pitch
      th5 = elbow roll 
      th6 = wrist pitch
      th7 = wrist roll 
*****/
using namespace angles;
using namespace bandit_ik;

PR2ArmIK::PR2ArmIK()
{
}

bool PR2ArmIK::init(const urdf::Model &robot_model, const std::string &root_name, const std::string &tip_name)
{
  std::vector<urdf::Pose> link_offset;
  int num_joints = 0;
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
  while(link && num_joints < 6)
  {
    boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(link->parent_joint->name);
    if(!joint)
    {
      ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
      return false;
    }
    if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      link_offset.push_back(link->parent_joint->parent_to_joint_origin_transform);
      angle_multipliers_.push_back(joint->axis.x*fabs(joint->axis.x) +  joint->axis.y*fabs(joint->axis.y) +  joint->axis.z*fabs(joint->axis.z));
      ROS_INFO("Joint axis: %d, %f, %f, %f",6-num_joints,joint->axis.x,joint->axis.y,joint->axis.z);
      if(joint->type != urdf::Joint::CONTINUOUS)
      {
        min_angles_.push_back(joint->safety->soft_lower_limit);
        max_angles_.push_back(joint->safety->soft_upper_limit);
        continuous_joint_.push_back(false);
      }
      else
      {
        min_angles_.push_back(-M_PI);
        max_angles_.push_back(M_PI);
        continuous_joint_.push_back(true);
      }
      addJointToChainInfo(link->parent_joint,solver_info_);
      num_joints++;
    }
    link = robot_model.getLink(link->getParent()->name);
  } 

  solver_info_.link_names.push_back(tip_name);

  //  solver_info_.link_names.push_back(tip_name);
  // We expect order from root to tip, so reverse the order
  std::reverse(angle_multipliers_.begin(),angle_multipliers_.end());
  std::reverse(min_angles_.begin(),min_angles_.end());
  std::reverse(max_angles_.begin(),max_angles_.end());
  std::reverse(link_offset.begin(),link_offset.end());
  std::reverse(solver_info_.limits.begin(),solver_info_.limits.end());
  std::reverse(solver_info_.joint_names.begin(),solver_info_.joint_names.end());
  std::reverse(solver_info_.link_names.begin(),solver_info_.link_names.end());
  std::reverse(continuous_joint_.begin(),continuous_joint_.end());

  if(num_joints != 6)
  {
    ROS_INFO("PR2ArmIK:: Chain from %s to %s does not have 6 joints",root_name.c_str(),tip_name.c_str());
    return false;
  }

  torso_shoulder_offset_x_ = link_offset[0].position.x;
  torso_shoulder_offset_y_ = link_offset[0].position.y;
  torso_shoulder_offset_z_ = link_offset[0].position.z;
  shoulder_upperarm_offset_ = distance(link_offset[1]);
  upperarm_elbow_offset_ = distance(link_offset[2]);
  elbow_wrist_offset_ = distance(link_offset[4]);
  shoulder_elbow_offset_ = shoulder_upperarm_offset_ + upperarm_elbow_offset_;
  shoulder_wrist_offset_ = shoulder_upperarm_offset_+upperarm_elbow_offset_+elbow_wrist_offset_;



  Eigen::Matrix4f home = Eigen::Matrix4f::Identity();
  home(0,3) = shoulder_upperarm_offset_ +  upperarm_elbow_offset_ +  elbow_wrist_offset_;
  home_inv_ = home.inverse();
  grhs_ = home;
  gf_ = home_inv_;
  solution_.resize( NUM_JOINTS_ARM7DOF);
  return true;
}

void PR2ArmIK::addJointToChainInfo(boost::shared_ptr<const urdf::Joint> joint, kinematics_msgs::KinematicSolverInfo &info)
{
  motion_planning_msgs::JointLimits limit;
  info.joint_names.push_back(joint->name);//Joints are coming in reverse order
  limit.min_position = joint->safety->soft_lower_limit;
  limit.max_position = joint->safety->soft_upper_limit;
  if(joint->type != urdf::Joint::CONTINUOUS)
  {
    limit.min_position = joint->safety->soft_lower_limit;
    limit.max_position = joint->safety->soft_upper_limit;
    limit.has_position_limits = true;
  }
  else
  {
    limit.min_position = -M_PI;
    limit.max_position = M_PI;
    limit.has_position_limits = false;
  }
  limit.max_velocity = joint->limits->velocity;
  limit.has_velocity_limits = 1;
  info.limits.push_back(limit);
}

void PR2ArmIK::getSolverInfo(kinematics_msgs::KinematicSolverInfo &info)
{
  info = solver_info_;
}


void PR2ArmIK::computeIKShoulderPan(const Eigen::Matrix4f &g_in, const double &t1_in)
{
//t1 = shoulder/turret pan is specified
  solution_ik_.clear();

  Eigen::Matrix4f g = g_in;
//First bring everything into the arm frame
  g(0,3) = g_in(0,3) - torso_shoulder_offset_x_;
  g(1,3) = g_in(1,3) - torso_shoulder_offset_y_;
  g(2,3) = g_in(2,3) - torso_shoulder_offset_z_;

  double t1 = angles::normalize_angle(t1_in);
  if(!checkJointLimits(t1,0))
    return;


  double cost1, cost2, cost3, cost4;
  double sint1, sint2, sint3, sint4;

  gf_ = g*home_inv_;

  cost1 = cos(t1);
  sint1 = sin(t1);

  double t2(0), t3(0), t4(0), t5(0), t6(0), t7(0);

  double at(0), bt(0), ct(0);

  double theta2[2],theta3[2],theta4[2],theta5[2],theta6[4],theta7[2]; 

  double sopx = shoulder_upperarm_offset_*cost1;
  double sopy = shoulder_upperarm_offset_*sint1;
  double sopz = 0;

  double x = g(0,3);
  double y = g(1,3);
  double z = g(2,3);

  double dx = x - sopx;
  double dy = y - sopy;
  double dz = z - sopz;

  double dd = dx*dx + dy*dy + dz*dz;

  double numerator = dd-shoulder_upperarm_offset_*shoulder_upperarm_offset_+2*shoulder_upperarm_offset_*shoulder_elbow_offset_-2*shoulder_elbow_offset_*shoulder_elbow_offset_+2*shoulder_elbow_offset_*shoulder_wrist_offset_-shoulder_wrist_offset_*shoulder_wrist_offset_;
  double denominator = 2*(shoulder_upperarm_offset_-shoulder_elbow_offset_)*(shoulder_elbow_offset_-shoulder_wrist_offset_);

  double acosTerm = numerator/denominator;

  if (acosTerm > 1.0 || acosTerm < -1.0)
    return;

  double acos_angle = acos(acosTerm);
 
  theta4[0] = acos_angle;
  theta4[1] = -acos_angle;

#ifdef DEBUG
  std::cout << "ComputeIK::theta3:" << numerator << "," << denominator << "," << std::endl << theta4[0] << std::endl;
#endif

  for(int jj =0; jj < 2; jj++)
  {
    t4 = theta4[jj];
    cost4 = cos(t4);
    sint4 = sin(t4);

#ifdef DEBUG
    std::cout << "t4 " << t4 << std::endl;
#endif
    if(std::isnan(t4))
      continue;

    if(!checkJointLimits(t4,3))
      continue;

    at = x*cost1+y*sint1-shoulder_upperarm_offset_;
    bt = -z;
    ct = -shoulder_upperarm_offset_ + shoulder_elbow_offset_ + (shoulder_wrist_offset_-shoulder_elbow_offset_)*cos(t4);

    if(!solveCosineEqn(at,bt,ct,theta2[0],theta2[1]))
      continue;

    for(int ii=0; ii < 2; ii++)
    {
      t2 = theta2[ii];
      if(!checkJointLimits(t2,1))
        continue;


#ifdef DEBUG
      std::cout << "t2 " << t2 << std::endl; 
#endif
      sint2 = sin(t2);
      cost2 = cos(t2);

      at = sint1*(shoulder_elbow_offset_ - shoulder_wrist_offset_)*sint2*sint4;
      bt = (-shoulder_elbow_offset_+shoulder_wrist_offset_)*cost1*sint4;
      ct = y - (shoulder_upperarm_offset_+cost2*(-shoulder_upperarm_offset_+shoulder_elbow_offset_+(-shoulder_elbow_offset_+shoulder_wrist_offset_)*cos(t4)))*sint1;
      if(!solveCosineEqn(at,bt,ct,theta3[0],theta3[1]))
        continue;

      for(int kk =0; kk < 2; kk++)
      {           
        t3 = theta3[kk];

        if(!checkJointLimits(angles::normalize_angle(t3),2))
          continue;

        sint3 = sin(t3);
        cost3 = cos(t3);
#ifdef DEBUG
        std::cout << "t3 " << t3 << std::endl; 
#endif
        if(fabs((shoulder_upperarm_offset_-shoulder_elbow_offset_+(shoulder_elbow_offset_-shoulder_wrist_offset_)*cost4)*sint2+(shoulder_elbow_offset_-shoulder_wrist_offset_)*cost2*cost3*sint4-z) > IK_EPS )
          continue;

        if(fabs((shoulder_elbow_offset_-shoulder_wrist_offset_)*sint1*sint3*sint4+cost1*(shoulder_upperarm_offset_+cost2*(-shoulder_upperarm_offset_+shoulder_elbow_offset_+(-shoulder_elbow_offset_+shoulder_wrist_offset_)*cost4)+(shoulder_elbow_offset_-shoulder_wrist_offset_)*cost3*sint2*sint4) - x) > IK_EPS)
          continue;

        grhs_(0,0) = cost4*(gf_(0,0)*cost1*cost2+gf_(1,0)*cost2*sint1-gf_(2,0)*sint2)-(gf_(2,0)*cost2*cost3 + cost3*(gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2 + (-(gf_(1,0)*cost1) + gf_(0,0)*sint1)*sint3)*sint4;

        grhs_(0,1) = cost4*(gf_(0,1)*cost1*cost2 + gf_(1,1)*cost2*sint1 - gf_(2,1)*sint2) - (gf_(2,1)*cost2*cost3 + cost3*(gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2 + (-(gf_(1,1)*cost1) + gf_(0,1)*sint1)*sint3)*sint4;

        grhs_(0,2) = cost4*(gf_(0,2)*cost1*cost2 + gf_(1,2)*cost2*sint1 - gf_(2,2)*sint2) - (gf_(2,2)*cost2*cost3 + cost3*(gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2 + (-(gf_(1,2)*cost1) + gf_(0,2)*sint1)*sint3)*sint4;

        grhs_(1,0) = cost3*(gf_(1,0)*cost1 - gf_(0,0)*sint1) + gf_(2,0)*cost2*sint3 + (gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2*sint3;

        grhs_(1,1) = cost3*(gf_(1,1)*cost1 - gf_(0,1)*sint1) + gf_(2,1)*cost2*sint3 + (gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2*sint3;

        grhs_(1,2) = cost3*(gf_(1,2)*cost1 - gf_(0,2)*sint1) + gf_(2,2)*cost2*sint3 + (gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2*sint3;

        grhs_(2,0) = cost4*(gf_(2,0)*cost2*cost3 + cost3*(gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2 + (-(gf_(1,0)*cost1) + gf_(0,0)*sint1)*sint3) + (gf_(0,0)*cost1*cost2 + gf_(1,0)*cost2*sint1 - gf_(2,0)*sint2)*sint4;

        grhs_(2,1) = cost4*(gf_(2,1)*cost2*cost3 + cost3*(gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2 + (-(gf_(1,1)*cost1) + gf_(0,1)*sint1)*sint3) + (gf_(0,1)*cost1*cost2 + gf_(1,1)*cost2*sint1 - gf_(2,1)*sint2)*sint4;

        grhs_(2,2) = cost4*(gf_(2,2)*cost2*cost3 + cost3*(gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2 + (-(gf_(1,2)*cost1) + gf_(0,2)*sint1)*sint3) + (gf_(0,2)*cost1*cost2 + gf_(1,2)*cost2*sint1 - gf_(2,2)*sint2)*sint4;


        double val1 = sqrt(grhs_(0,1)*grhs_(0,1)+grhs_(0,2)*grhs_(0,2));
        double val2 = grhs_(0,0);

        theta6[0] = atan2(val1,val2);
        theta6[1] = atan2(-val1,val2);

//            theta6[3] = M_PI + theta6[0];
//            theta6[4] = M_PI + theta6[1];

        for(int mm = 0; mm < 2; mm++)
        {
          t6 = theta6[mm];
          if(!checkJointLimits(angles::normalize_angle(t6),5))
            continue;

#ifdef DEBUG
          std::cout << "t6 " << t6 << std::endl;
#endif
          if(fabs(cos(t6) - grhs_(0,0)) > IK_EPS)
            continue;

          if(fabs(sin(t6)) < IK_EPS)
          {
            //                std::cout << "Singularity" << std::endl;
            theta5[0] = acos(grhs_(1,1))/2.0;
            theta7[0] = theta7[0];
            theta7[1] = M_PI+theta7[0];
            theta5[1] = theta7[1];
          }
          else
          {
            theta7[0] = atan2(grhs_(0,1),grhs_(0,2));
            theta5[0] = atan2(grhs_(1,0),-grhs_(2,0));
            theta7[1] = M_PI+theta7[0];
            theta5[1] = M_PI+theta5[0];
          }
#ifdef DEBUG
          std::cout << "theta1: " << t1 << std::endl;
          std::cout << "theta2: " << t2 << std::endl;
          std::cout << "theta3: " << t3 << std::endl;
          std::cout << "theta4: " << t4 << std::endl;
          std::cout << "theta5: " << t5 << std::endl;
          std::cout << "theta6: " << t6 << std::endl;
          std::cout << "theta7: " << t7 << std::endl << std::endl << std::endl;
#endif
          for(int lll =0; lll < 2; lll++)
          {
            t5 = theta5[lll];
            t7 = theta7[lll];
            if(!checkJointLimits(t5,4))
						{
							//ROS_INFO( "!checkJointLimits(t5,4)");
              continue;
						}
/*
            if(!checkJointLimits(t7,6))
						{
							//ROS_INFO( "!checkJointLimits(t7,6)");
              continue;
						}
*/
#ifdef DEBUG
            std::cout << "t5" << t5 << std::endl;
            std::cout << "t7" << t7 << std::endl;
#endif      
            if(fabs(sin(t6)*sin(t7)-grhs_(0,1)) > IK_EPS || fabs(cos(t7)*sin(t6)-grhs_(0,2)) > IK_EPS)
              continue;

            solution_[0] = normalize_angle(t1)*angle_multipliers_[0];
            solution_[1] = normalize_angle(t2)*angle_multipliers_[1];
            solution_[2] = normalize_angle(t3)*angle_multipliers_[2];
            solution_[3] = normalize_angle(t4)*angle_multipliers_[3];
            solution_[4] = normalize_angle(t5)*angle_multipliers_[4];
            solution_[5] = normalize_angle(t6)*angle_multipliers_[5];
            //solution_[6] = normalize_angle(t7)*angle_multipliers_[6];
            solution_ik_.push_back(solution_);

#ifdef DEBUG
            std::cout << "SOLN " << solution_[0] << " " << solution_[1] << " " <<  solution_[2] << " " << solution_[3] <<  " " << solution_[4] << " " << solution_[5] <<  " " << solution_[6] << std::endl << std::endl;
#endif
          }
        }
      }
    }
  }
}


void PR2ArmIK::computeIKShoulderRoll(const Eigen::Matrix4f &g_in, const double &t3)
{
	ROS_INFO( "ShoulderRoll" );
  solution_ik_.clear();
//t3 = shoulder/turret roll is specified
  Eigen::Matrix4f g = g_in;
//First bring everything into the arm frame
  g(0,3) = g_in(0,3) - torso_shoulder_offset_x_;
  g(1,3) = g_in(1,3) - torso_shoulder_offset_y_;
  g(2,3) = g_in(2,3) - torso_shoulder_offset_z_;

	ROS_INFO( "tso: %0.4f %0.4f %0.4f", torso_shoulder_offset_x_, torso_shoulder_offset_y_, torso_shoulder_offset_z_ );
	ROS_INFO( "g: %0.2f %0.2f %0.2f", g(0,3), g(1,3), g(2,3) );

  if(!checkJointLimits(t3,2))
  {
		ROS_INFO( "check joint limits failed" );
    return;
  }
  double x = g(0,3);
  double y = g(1,3);
  double z = g(2,3);
  double cost1, cost2, cost3, cost4;
  double sint1, sint2, sint3, sint4;

  gf_ = g*home_inv_;

  cost3 = cos(t3);
  sint3 = sin(t3);

  double t1(0), t2(0), t4(0), t5(0), t6(0), t7(0);

  double at(0), bt(0), ct(0);

  double theta1[2],theta2[2],theta4[4],theta5[2],theta6[4],theta7[2];

  double c0 = -sin(-t3)*elbow_wrist_offset_;
  double c1 = -cos(-t3)*elbow_wrist_offset_;

  double d0 = 4*shoulder_upperarm_offset_*shoulder_upperarm_offset_*(upperarm_elbow_offset_*upperarm_elbow_offset_+c1*c1-z*z);
  double d1 = 8*shoulder_upperarm_offset_*shoulder_upperarm_offset_*upperarm_elbow_offset_*elbow_wrist_offset_;
  double d2 = 4*shoulder_upperarm_offset_*shoulder_upperarm_offset_*(elbow_wrist_offset_*elbow_wrist_offset_-c1*c1);

  double b0 = x*x+y*y+z*z-shoulder_upperarm_offset_*shoulder_upperarm_offset_-upperarm_elbow_offset_*upperarm_elbow_offset_-c0*c0-c1*c1;
  double b1 = -2*upperarm_elbow_offset_*elbow_wrist_offset_;

	ROS_INFO( "su: %0.4f ue: %0.4f ew: %0.4f", shoulder_upperarm_offset_, upperarm_elbow_offset_, elbow_wrist_offset_ );
	ROS_INFO( "a: %0.4f b: %0.4f c: %0.4f", b1*b1-d2,2*b0*b1-d1,b0*b0-d0 );

  if(!solveQuadratic(b1*b1-d2,2*b0*b1-d1,b0*b0-d0,&theta4[0],&theta4[1]))
  {
    ROS_INFO("No solution to quadratic eqn");
#ifdef DEBUG
    printf("No solution to quadratic eqn\n");
#endif
    return;
  }
  theta4[0] = acos(theta4[0]);
  theta4[2] = acos(theta4[1]);
  theta4[1] = -theta4[0];
  theta4[3] = -theta4[2];

	ROS_INFO( "for jj" );

  for(int jj = 0; jj < 4; jj++)
  {
    t4 = theta4[jj];

    if(!checkJointLimits(t4,2))
    {
			ROS_INFO( "!checkJointLimits(t4,2)");
      continue;
    }
    cost4 = cos(t4);
    sint4 = sin(t4);
#ifdef DEBUG
    std::cout << "t4 " << t4 << std::endl;
#endif
    if(std::isnan(t4))
		{
			ROS_INFO( "t4 nan" );
      continue;
		}
    at = cos(t3)*sin(t4)*(shoulder_elbow_offset_-shoulder_wrist_offset_);
    bt = (shoulder_upperarm_offset_-shoulder_elbow_offset_+(shoulder_elbow_offset_-shoulder_wrist_offset_)*cos(t4));
    ct = z;

    if(!solveCosineEqn(at,bt,ct,theta2[0],theta2[1]))
		{
			ROS_INFO("!solveCosEqn(theta2[0])");
      continue;
		}
    for(int ii=0; ii < 2; ii++)
    {
      t2 = theta2[ii];
#ifdef DEBUG
      std::cout << "t2 " << t2 << std::endl;
#endif
      if(!checkJointLimits(t2,1))
      {
        ROS_INFO( "!checkJointLimits(t2,1)");
        continue;
      }


      sint2 = sin(t2);
      cost2 = cos(t2);

      at = -y;
      bt = x;
      ct = (shoulder_elbow_offset_-shoulder_wrist_offset_)*sin(t3)*sin(t4);
      if(!solveCosineEqn(at,bt,ct,theta1[0],theta1[1]))
      {
#ifdef DEBUG
        std::cout << "could not solve cosine equation for t1" << std::endl;
#endif
        ROS_INFO( "could not solve cosine equation for t1" );
        continue;
      }

      for(int kk =0; kk < 2; kk++)
      {           
        t1 = theta1[kk];
#ifdef DEBUG
        std::cout << "t1 " << t1 << std::endl;
#endif
        if(!checkJointLimits(t1,0))
        {
          ROS_INFO( "!checkJointLimits(t1,0)");
          continue;
        }
        sint1 = sin(t1);
        cost1 = cos(t1);
        if(fabs((shoulder_upperarm_offset_-shoulder_elbow_offset_+(shoulder_elbow_offset_-shoulder_wrist_offset_)*cost4)*sint2+(shoulder_elbow_offset_-shoulder_wrist_offset_)*cost2*cost3*sint4-z) > IK_EPS )
        {
#ifdef DEBUG
          printf("z value not matched %f\n",fabs((shoulder_upperarm_offset_-shoulder_elbow_offset_+(shoulder_elbow_offset_-shoulder_wrist_offset_)*cost4)*sint2+(shoulder_elbow_offset_-shoulder_wrist_offset_)*cost2*cost3*sint4-z));
#endif
          ROS_INFO("z value not matched");
          continue;
        }
        if(fabs((shoulder_elbow_offset_-shoulder_wrist_offset_)*sint1*sint3*sint4+cost1*(shoulder_upperarm_offset_+cost2*(-shoulder_upperarm_offset_+shoulder_elbow_offset_+(-shoulder_elbow_offset_+shoulder_wrist_offset_)*cost4)+(shoulder_elbow_offset_-shoulder_wrist_offset_)*cost3*sint2*sint4) - x) > IK_EPS)
        {
#ifdef DEBUG
          printf("x value not matched by %f\n",fabs((shoulder_elbow_offset_-shoulder_wrist_offset_)*sint1*sint3*sint4+cost1*(shoulder_upperarm_offset_+cost2*(-shoulder_upperarm_offset_+shoulder_elbow_offset_+(-shoulder_elbow_offset_+shoulder_wrist_offset_)*cost4)+(shoulder_elbow_offset_-shoulder_wrist_offset_)*cost3*sint2*sint4) - x));
#endif
          ROS_INFO("x value not matched");
          continue;
        }
        if(fabs(-(shoulder_elbow_offset_-shoulder_wrist_offset_)*cost1*sint3*sint4+sint1*(shoulder_upperarm_offset_+cost2*(-shoulder_upperarm_offset_+shoulder_elbow_offset_+(-shoulder_elbow_offset_+shoulder_wrist_offset_)*cost4)+(shoulder_elbow_offset_-shoulder_wrist_offset_)*cost3*sint2*sint4) - y) > IK_EPS)
        {
#ifdef DEBUG
          printf("y value not matched\n");
#endif
          ROS_INFO("y value not matched");
          continue;
        }
        grhs_(0,0) = cost4*(gf_(0,0)*cost1*cost2+gf_(1,0)*cost2*sint1-gf_(2,0)*sint2)-(gf_(2,0)*cost2*cost3 + cost3*(gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2 + (-(gf_(1,0)*cost1) + gf_(0,0)*sint1)*sint3)*sint4;

        grhs_(0,1) = cost4*(gf_(0,1)*cost1*cost2 + gf_(1,1)*cost2*sint1 - gf_(2,1)*sint2) - (gf_(2,1)*cost2*cost3 + cost3*(gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2 + (-(gf_(1,1)*cost1) + gf_(0,1)*sint1)*sint3)*sint4;

        grhs_(0,2) = cost4*(gf_(0,2)*cost1*cost2 + gf_(1,2)*cost2*sint1 - gf_(2,2)*sint2) - (gf_(2,2)*cost2*cost3 + cost3*(gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2 + (-(gf_(1,2)*cost1) + gf_(0,2)*sint1)*sint3)*sint4;

        grhs_(1,0) = cost3*(gf_(1,0)*cost1 - gf_(0,0)*sint1) + gf_(2,0)*cost2*sint3 + (gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2*sint3;

        grhs_(1,1) = cost3*(gf_(1,1)*cost1 - gf_(0,1)*sint1) + gf_(2,1)*cost2*sint3 + (gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2*sint3;

        grhs_(1,2) = cost3*(gf_(1,2)*cost1 - gf_(0,2)*sint1) + gf_(2,2)*cost2*sint3 + (gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2*sint3;

        grhs_(2,0) = cost4*(gf_(2,0)*cost2*cost3 + cost3*(gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2 + (-(gf_(1,0)*cost1) + gf_(0,0)*sint1)*sint3) + (gf_(0,0)*cost1*cost2 + gf_(1,0)*cost2*sint1 - gf_(2,0)*sint2)*sint4;

        grhs_(2,1) = cost4*(gf_(2,1)*cost2*cost3 + cost3*(gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2 + (-(gf_(1,1)*cost1) + gf_(0,1)*sint1)*sint3) + (gf_(0,1)*cost1*cost2 + gf_(1,1)*cost2*sint1 - gf_(2,1)*sint2)*sint4;

        grhs_(2,2) = cost4*(gf_(2,2)*cost2*cost3 + cost3*(gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2 + (-(gf_(1,2)*cost1) + gf_(0,2)*sint1)*sint3) + (gf_(0,2)*cost1*cost2 + gf_(1,2)*cost2*sint1 - gf_(2,2)*sint2)*sint4;









        double val1 = sqrt(grhs_(0,1)*grhs_(0,1)+grhs_(0,2)*grhs_(0,2));
        double val2 = grhs_(0,0);

        theta6[0] = atan2(val1,val2);
        theta6[1] = atan2(-val1,val2);

        for(int mm = 0; mm < 2; mm++)
        {
          t6 = theta6[mm];
#ifdef DEBUG
          std::cout << "t6 " << t6 << std::endl;
#endif
          if(!checkJointLimits(t6,5))
          {
						ROS_INFO( "!checkJointLimits(t6,5)");
            continue;
          }


          if(fabs(cos(t6) - grhs_(0,0)) > IK_EPS)
					{
						ROS_INFO( "fabs(cos" );
            continue;
					}
          if(fabs(sin(t6)) < IK_EPS)
          {
            //                std::cout << "Singularity" << std::endl;
            theta5[0] = acos(grhs_(1,1))/2.0;
            theta7[0] = theta5[0];
//            theta7[1] = M_PI+theta7[0];
//            theta5[1] = theta7[1];
          }
          else
          {
            theta7[0] = atan2(grhs_(0,1)/sin(t6),grhs_(0,2)/sin(t6));
            theta5[0] = atan2(grhs_(1,0)/sin(t6),-grhs_(2,0)/sin(t6));
//            theta7[1] = M_PI+theta7[0];
//            theta5[1] = M_PI+theta5[0];
          }
          for(int lll =0; lll < 1; lll++)
          {
            t5 = theta5[lll];
            t7 = theta7[lll];

            if(!checkJointLimits(t5,4))
            {
						ROS_INFO( "!checkJointLimits(t5,4)");
              continue;
            }
/*
            if(!checkJointLimits(t7,6))
            {
						ROS_INFO( "!checkJointLimits(t7,6)");
              continue;
            }
*/

#ifdef DEBUG
            std::cout << "t5 " << t5 << std::endl;
            std::cout << "t7 " << t7 << std::endl;
#endif      
            //           if(fabs(sin(t6)*sin(t7)-grhs_(0,1)) > IK_EPS || fabs(cos(t7)*sin(t6)-grhs_(0,2)) > IK_EPS)
            //  continue;

#ifdef DEBUG
            std::cout << "theta1: " << t1 << std::endl;
            std::cout << "theta2: " << t2 << std::endl;
            std::cout << "theta3: " << t3 << std::endl;
            std::cout << "theta4: " << t4 << std::endl;
            std::cout << "theta5: " << t5 << std::endl;
            std::cout << "theta6: " << t6 << std::endl;
            std::cout << "theta7: " << t7 << std::endl << std::endl << std::endl;
#endif


            solution_[0] = normalize_angle(t1*angle_multipliers_[0]);
            solution_[1] = normalize_angle(t2*angle_multipliers_[1]);
            solution_[2] = t3*angle_multipliers_[2];
            solution_[3] = normalize_angle(t4*angle_multipliers_[3]);
            solution_[4] = normalize_angle(t5*angle_multipliers_[4]);
            solution_[5] = normalize_angle(t6*angle_multipliers_[5]);
            //solution_[6] = normalize_angle(t7*angle_multipliers_[6]);
            solution_ik_.push_back(solution_);
            std::cout << "SOLN " << solution_[0] << " " << solution_[1] << " " <<  solution_[2] << " " << solution_[3] <<  " " << solution_[4] << " " << solution_[5] <<  " " << solution_[6] << std::endl << std::endl;
#ifdef DEBUG
            std::cout << "SOLN " << solution_[0] << " " << solution_[1] << " " <<  solution_[2] << " " << solution_[3] <<  " " << solution_[4] << " " << solution_[5] <<  " " << solution_[6] << std::endl << std::endl;
#endif
          }
        }
      }
    }
  }
	ROS_INFO( "end ShoulderRoll" );
}


bool PR2ArmIK::checkJointLimits(const std::vector<double> &joint_values)
{
  for(int i=0; i<NUM_JOINTS_ARM7DOF; i++)
  {
    if(!checkJointLimits(angles::normalize_angle(joint_values[i]*angle_multipliers_[i]),i))
    {
      return false;
    }
  }
  return true;
}

bool  PR2ArmIK::checkJointLimits(const double &joint_value, const int &joint_num)
{
  double jv;
  if(continuous_joint_[joint_num])
    jv= angles::normalize_angle(joint_value*angle_multipliers_[joint_num]);
  else if (joint_num ==1)
    jv = joint_value*angle_multipliers_[joint_num];
  else
    jv= angles::normalize_angle(joint_value*angle_multipliers_[joint_num]);

	//if( joint_num == 3 ) jv = M_PI - jv;
	//if( joint_num == 5 ) jv -= M_PI;

  if(jv < min_angles_[joint_num] || jv > max_angles_[joint_num])
  {
     ROS_INFO("Angle %d = %f out of range: (%f,%f)",joint_num,joint_value,min_angles_[joint_num],max_angles_[joint_num]);
    return false;
  }
  return true;
}
