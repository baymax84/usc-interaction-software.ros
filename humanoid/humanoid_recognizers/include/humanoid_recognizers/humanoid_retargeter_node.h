/***************************************************************************
 *  include/humanoid_recognizers/humanoid_retargeter_node.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
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
 *  * Neither the name of usc-ros-pkg nor the names of its
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

#ifndef HUMANOIDRECOGNIZERS_HUMANOIDRETARGETERNODE_H_
#define HUMANOIDRECOGNIZERS_HUMANOIDRETARGETERNODE_H_

#include <urdf/model.h>

#include <quickdev/node.h>
#include <quickdev/reconfigure_policy.h>

#include <humanoid_recognizers/humanoid_recognizer_policy.h>

#include <humanoid/spatial_metrics.h>

#include <tf_conversions/tf_kdl.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <sensor_msgs/JointState.h>

typedef sensor_msgs::JointState _JointStateMsg;
typedef HumanoidRecognizerPolicy<_JointStateMsg> _HumanoidRecognizerPolicy;
using humanoid::Humanoid;

template
<
    class __FkSolverPos = KDL::ChainFkSolverPos_recursive,
    class __IkSolverVel = KDL::ChainIkSolverVel_pinv,
    class __IkSolverPos = KDL::ChainIkSolverPos_NR_JL
>
struct IkChainSolver
{
    KDL::Chain target_chain_;
    __FkSolverPos fk_pos_solver_;
    __IkSolverVel ik_vel_solver_;
    __IkSolverPos ik_pos_solver_;

    template<class... __Args>
    IkChainSolver( decltype( target_chain_ ) const & target_chain, const KDL::JntArray & q_min , const KDL::JntArray & q_max,__Args... args )
    :
        target_chain_( target_chain ),
        fk_pos_solver_( target_chain_ ),
        ik_vel_solver_( target_chain_ ),
        ik_pos_solver_( target_chain_, q_min, q_max, fk_pos_solver_, ik_vel_solver_, target_chain_.getNrOfJoints(), args... )
    {
        //
    }

    template<class... __Args>
    IkChainSolver( KDL::Tree const & tree, const KDL::JntArray & q_min, const KDL::JntArray & q_max, std::string const & start_frame, std::string const & end_frame, __Args... args )
    :
        target_chain_( getChainFromTree( tree, start_frame, end_frame ) ),
        fk_pos_solver_( target_chain_ ),
        ik_vel_solver_( target_chain_ ),
        ik_pos_solver_( target_chain_, q_min, q_max, fk_pos_solver_, ik_vel_solver_, target_chain_.getNrOfJoints(), args... )
    {
        // 
    }

    static KDL::Chain getChainFromTree( KDL::Tree const & tree, std::string const & start_frame, std::string const & end_frame )
    {
        KDL::Chain chain;
        tree.getChain( start_frame, end_frame, chain );
        return chain;
    }

    KDL::JntArray solveForFrame( KDL::JntArray const & initial_configuration, KDL::Frame const & target_frame )
    {
        KDL::JntArray result_configuration;

        ik_pos_solver_.CartToJnt( initial_configuration, target_frame, result_configuration );

        return result_configuration;
    }
};

template<class __IkChainSolver>
struct HumanoidChainRetargeter : __IkChainSolver
{
    std::string from_joint_;
    std::string to_joint_;

    template<class... __Args>
    HumanoidChainRetargeter( std::string const & from_joint, std::string const & to_joint, __Args... args )
    :
        __IkChainSolver( args... ),
        from_joint_( from_joint ),
        to_joint_( to_joint )
    {
        //
    }

    _JointStateMsg solvePose( Humanoid const & humanoid )
    {
        static btQuaternion const zero_quat( 0, 0, 0, 1 );
        auto const target_pos = humanoid::spatial::getDistance( humanoid[from_joint_], humanoid[to_joint_] );

        KDL::Frame target_frame;
        tf::TransformTFToKDL( btTransform( zero_quat, target_pos ), target_frame );

        // needs to be implemented
//        auto const current_state = humanoid.getJointStateChain( from_joint_, to_joint_ );
        // needs to be implemented
//        KDL::JntArray current_configuration = unit::make_unit( current_state );

//        auto const result_configuration = __IkChainSolver::solveForFrame( current_configuration, target_frame );

        _JointStateMsg result;
/*
        result.names = current_state.names;
        result.positions.resize( result_configuration.rows() );
        std::copy( result_configuration( 0 ), result_configuration( result_configuration.rows() - 1 ), result.positions.begin() );
*/
        return result;
    }
};

typedef HumanoidChainRetargeter<IkChainSolver<> > _HumanoidChainRetargeter;

QUICKDEV_DECLARE_NODE( HumanoidRetargeter, _HumanoidRecognizerPolicy )

QUICKDEV_DECLARE_NODE_CLASS( HumanoidRetargeter )
{
    urdf::Model model_;

    KDL::Tree kinematic_tree_;

    std::map<std::string, boost::shared_ptr<_HumanoidChainRetargeter> > retargeters_;

    KDL::JntArray q_min_;
    KDL::JntArray q_max_;
    KDL::JntArray L_qmin_;
    KDL::JntArray R_qmin_;
    KDL::JntArray L_qmax_;
    KDL::JntArray R_qmax_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( HumanoidRetargeter )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        // get nodehandle
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        // load and parse urdf
        //For Nao
        if(!model_.initFile("/home/matias/ros_workspace/stacks/nao_common/nao_description/urdf/nao_v4.urdf"))
        {
          ROS_ERROR("Failed to Parse URDF");
        } 
 
        std::cout << "Loading URDF File" << std::endl;

        // get uri of urdf file
        /**auto const urdf_uri = ros::ParamReader<std::string, 1>::readParam( nh_rel, "urdf_uri", "" );
        std::cout << "URDI: " << urdf_uri << std::endl; 

        // load and parse document at specified uri
        std::cout << "Loading xml document" << std::endl;
        TiXmlDocument urdf_doc( urdf_uri );
        urdf_doc.LoadFile();

        // create our robot model from the "robot" element of the document
        std::cout << "Building urdf model from xml" << std::endl;
        model_.initXml( urdf_doc.FirstChildElement( "robot" ) );**/

        std::cout << "Read " << model_.joints_.size() << " joints" << std::endl;
        for( auto joint_it = model_.joints_.cbegin(); joint_it != model_.joints_.cend(); ++joint_it )
        {     
            std::cout << joint_it->first << std::endl;
        }
        std::cout << "--------------------" << std::endl;

        


        /**for(int i = 0; i < model_.joints_.size(); i++)
        {
            q_min_.data[i] = model_.joints_[i]->limits->lower;
            q_max_.data[i] = model_.joints_[i]->limits->upper;
            std::cout << model_.joints_[i]->name << "-->" << q_min_.data[i] << "to" << q_max_.data[i] << std::endl;
        }
        std::cout << "--------------------" << std::endl;**/

        std::cout << "Read " << model_.links_.size() << " links" << std::endl;
        for( auto link_it = model_.links_.cbegin(); link_it != model_.links_.cend(); ++link_it )
        {
            std::cout << link_it->first << std::endl;
        }

        // prepare kinematic chains for the robot's arms
        // create a KDL::Tree from our robot model
        std::cout << "Creating kinematic tree from urdf" << std::endl;
        kdl_parser::treeFromUrdfModel( model_, kinematic_tree_ );

        auto const num_segments = kinematic_tree_.getNrOfSegments();

        std::cout << "Tree has " << num_segments << " segments" << std::endl;
        auto const & segment_map = kinematic_tree_.getSegments();
        for( auto segment_it = segment_map.cbegin(); segment_it != segment_map.cend(); ++segment_it )
        {
            std::cout << segment_it->first << std::endl;
        }
        std::cout << "--------------------" << std::endl;

        KDL::Chain l_arm_chain;
        KDL::Chain r_arm_chain;
        if(!kinematic_tree_.getChain( "torso", "LWristYaw_link", l_arm_chain ))
        {
           ROS_ERROR("Left Arm Chain did not initialize properly");
        }

        if(!kinematic_tree_.getChain( "torso", "RWristYaw_link", r_arm_chain ))
        {
           ROS_ERROR("Right Arm Chain did not initialize properly");
        }

        std::cout << "Chain from torso to lhand has " << l_arm_chain.getNrOfSegments() << " segments" << std::endl;
        for( auto chain_it = l_arm_chain.segments.cbegin(); chain_it != l_arm_chain.segments.cend(); ++ chain_it )
        {
            auto const & joint = chain_it->getJoint();
            std::cout << chain_it->getName() << " -> " << joint.getName() << " [" << joint.getTypeName() << "]" << std::endl;
        }
        std::cout << "--------------------" << std::endl;

        std::cout << "Chain from torso to rhand has " << l_arm_chain.getNrOfSegments() << " segments" << std::endl;
        for( auto chain_it = r_arm_chain.segments.cbegin(); chain_it != r_arm_chain.segments.cend(); ++ chain_it )
        {
            auto const & joint = chain_it->getJoint();
            std::cout << chain_it->getName() << " -> " << joint.getName() << " [" << joint.getTypeName() << "]" << std::endl;
        }
        std::cout << "--------------------" << std::endl;

        std::cout << "Reading Right Joint Limits" << std::endl;

        //Resize q_min array and q_max array
        R_qmin_.resize(6);//Hard coded 6 for speed and simplicity.  Later dynamic sizing should be used
        R_qmax_.resize(6);   

        boost::shared_ptr<const urdf::Link> link = model_.getLink("RWristYaw_link"); 
        boost::shared_ptr<const urdf::Joint> joint = model_.getJoint(link->parent_joint->name);

        std::cout <<"For loop initializes" << std::endl;      
        for(unsigned int i = 0; i < 6; i++) //Hard coded 6 for speed and simplicity.  Later dynamic sizing should be used
        {
            joint = model_.getJoint(link->parent_joint->name);

//            causing compiler issues "-fpermissive"
//            std::cout << link->parent_joint->name << " pointer: " << &model_.getLink(link->parent_joint->name) << std::endl;
            std::cout << joint->name << " pointer: " << &joint << std::endl;

            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
            {
                std::cout << joint->name.c_str() << ": " << joint->limits->lower << " to " << joint->limits->upper << std::endl;

                float lower, upper;
                if ( joint->type != urdf::Joint::CONTINUOUS ) 
                {
                    lower = joint->limits->lower;
                    upper = joint->limits->upper;
                }
                else
                {
                    lower = -.5;
                    upper = .5;
                }

                int index = 6 - i - 1; //Selected 6 for simplicity sake later dynamic allocation should be used

                R_qmin_.data[index] = lower;
                R_qmax_.data[index] = upper;
            }

            std::cout << "Made it through if statement!" << std::endl;       

            link = model_.getLink(link->getParent()->name);

            std::cout << "Loop Iteration!" << std::endl;
        }
        std::cout << "--------------------" << std::endl;
        std::cout << "Reading Right Joint Limits" << std::endl;

        // Reading Joint limits from URDF for left and right arm to set up JntArray that holds Joint limits
        //  _HumanoidChainRetargeter initialization

        // Reinitialize L_qmin/max joint arrays
        L_qmin_.resize(6);
        L_qmax_.resize(6);  //Hard coded 6 for speed and simplicity.  Later dynamic sizing should be used

        link = model_.getLink("LWristYaw_link"); 
        joint = model_.getJoint(link->parent_joint->name);

        std::cout <<"For loop initializes" << std::endl; 
        
        for(unsigned int i = 0; i < 6; i++) //Hard coded 6 for speed and simplicity.  Later dynamic sizing should be used
        {
            joint = model_.getJoint(link->parent_joint->name);

//            causing compiler issues "-fpermissive"
//            std::cout << link->parent_joint->name << " pointer: " << &model_.getLink(link->parent_joint->name) << std::endl;
            std::cout << joint->name << " pointer: " << &joint << std::endl;

            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
            {
                std::cout << joint->name.c_str() << ": " << joint->limits->lower << " to " << joint->limits->upper << std::endl;

                float lower, upper;
                if ( joint->type != urdf::Joint::CONTINUOUS ) 
                {
                    lower = joint->limits->lower;
                    upper = joint->limits->upper;
                }
                else
                {
                    lower = -.5;
                    upper = .5;
                }

                int index = 6 - i - 1;//Hard coded 6 for speed and simplicity.  Later dynamic sizing should be used

                L_qmin_.data[index] = lower;
                L_qmax_.data[index] = upper;
            }

            std::cout << "Made it through if statement!" << std::endl;       

            link = model_.getLink(link->getParent()->name);

            std::cout << "Loop Iteration!" << std::endl;
        }
        std::cout << "--------------------" << std::endl;


        // for each pair of joints { left_shoulder -> left_elbow, left_elbow -> left_wrist, right_shoulder -> right_elbow,
        // right_elbow -> right_wrist }
        // set up an ik solver for the target platform's corresponding joint (in this version we only care about angles 
        // so we need to normalize the link lengths on one platform)

       
        // Torso_link -> [L/R]ShoulderPitch -> [L/R]ShoulderPitch_link -> [L/R]ShoulderRoll -> [L/R]ShoulderRoll_link -> [L/R]ElbowYaw -> ...
        /**retargeters_["l_shoulder_ori"] = quickdev::make_shared( new _HumanoidChainRetargeter( "left_shoulder", "left_elbow", kinematic_tree_, "LShoulderPitch_link", "LShoulderRoll_link" ) );
        retargeters_["r_shoulder_ori"] = quickdev::make_shared( new _HumanoidChainRetargeter( "right_shoulder", "right_elbow", kinematic_tree_, "RShoulderPitch_link", "RShoulderRoll_link" ) );**/

        // ... -> [L/R]ShoulderRoll_link -> [L/R]ElbowYaw -> [L/R]ElbowYaw_link -> [L/R]ElbowRoll -> [L/R]ElbowRoll_link -> [L/R]WristYaw -> ...
        /**retargeters_["l_elbow_ori"] = quickdev::make_shared( new _HumanoidChainRetargeter( "left_elbow", "left_wrist", kinematic_tree_, "LShoulderRoll_link", "LElbowRoll_link" ) );
        retargeters_["r_elbow_ori"] = quickdev::make_shared( new _HumanoidChainRetargeter( "right_elbow", "right_wrist", kinematic_tree_, "RShoulderRoll_link", "RElbowRoll_link" ) );**/

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        auto const & humanoids = _HumanoidRecognizerPolicy::getHumanoids();
        if( humanoids.size() == 0 ) return;

        auto const target_humanoid = *humanoids.cbegin();

        // calculate the position of the target joint on the source humanoid

        auto const l_elbow_pos = humanoid::spatial::getDistance( target_humanoid["left_shoulder"], target_humanoid["left_elbow"] );
        auto const r_elbow_pos = humanoid::spatial::getDistance( target_humanoid["right_shoulder"], target_humanoid["right_elbow"] );

        auto const l_wrist_pos = humanoid::spatial::getDistance( target_humanoid["left_elbow"], target_humanoid["left_wrist"] );
        auto const r_wrist_pos = humanoid::spatial::getDistance( target_humanoid["right_elbow"], target_humanoid["right_wrist"] );

        // run the ik solver for the target platform
/*
        for( auto retargeter = retargeters_.begin(); solver != retargeters_.end(); ++retargeter )
        {
            retargeter->
        }
*/
        // publish the joint angles calculated by the ik solver
    }
};

#endif // HUMANOIDRECOGNIZERS_HUMANOIDRETARGETERNODE_H_
