// bsd license blah blah
#include <cstring>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "robot_state_publisher/treefksolverposfull_recursive.hpp"
#include <string>
#include <map>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <bandit_ik/bandit_arm_kinematics_utils.h>
#include "geometry_msgs/Transform.h"
using std::string;
using std::map;
using std::make_pair;
using namespace Eigen;
using namespace bandit_ik;

int num_joints = 0;

sensor_msgs::JointState g_js, g_actual_js;
ros::Publisher *g_joint_pub = NULL;
tf::TransformBroadcaster *g_tf_broadcaster = NULL;
tf::TransformListener *g_tf_listener = NULL;
KDL::TreeFkSolverPosFull_recursive *g_fk_solver = NULL;
KDL::ChainIkSolverPos_NR_JL *g_ik_solver = NULL;
tf::Transform g_target_origin, g_target;
std::vector<double> g_pose;
std::string root_name, tip_name;
KDL::JntArray q_min, q_max;
tf::Transform fk_tool(const std::vector<double> &x) // joint angles 
{
  map<string, double> joint_pos;
  for( int i = 0; i <  num_joints; i++ )
  {
    joint_pos.insert(make_pair(g_js.name[i], x[i]));
  }

  map<string, KDL::Frame> link_poses;
  g_fk_solver->JntToCart(joint_pos, link_poses);
  if (link_poses.size() < num_joints)
    ROS_ERROR("couldn't compute forward kinematics.");
  for (map<string, KDL::Frame>::const_iterator f = link_poses.begin();
      f != link_poses.end(); ++f)
  {
    if (f->first == tip_name)
    {
      tf::Transform tf_frame;
      tf::TransformKDLToTF(f->second, tf_frame);
      return tf_frame;
    }
  }
  return tf::Transform();
}

bool ik_tool(tf::Transform t, std::vector<double> &joints)
{
  // assumes that "joints" coming in is the initial vector
  if (joints.size() != num_joints)
  {
    ROS_ERROR("woah there. the joints vector is supposed to be the start pos");
    return false;
  }
  KDL::JntArray q_init(num_joints), q(num_joints);
  for (int i = 0; i < num_joints; i++)
    q_init.data[i] = joints[i];
  // populate F_dest from tf::Transform parameter
  KDL::Frame F_dest;
  tf::TransformTFToKDL(t, F_dest);
  if (g_ik_solver->CartToJnt(q_init, F_dest, q) < 0)
  {
    ROS_ERROR("ik solver fail");
    return false;
  }
  for (int i = 0; i < num_joints; i++)
    joints[i] = q.data[i];
  return true;
}

void joint_cb(const sensor_msgs::JointState &msg)
{
  
  for( int i = 0; i < msg.name.size(); i++ )
  {
    for( int j = 0; j < num_joints; j++ )
    {
      if( msg.name[i] == g_js.name[j] )
      {
        g_actual_js.name[j] = g_js.name[j];
        g_actual_js.position[j] = msg.position[i];
      }
    }
  }
}

void target_cb(const geometry_msgs::TransformStamped &t_msg)
{
  // assume the transform coming in is a delta away from the center 
  // of our workspace (for now, at least)
  
  tf::StampedTransform t;
  std::vector<double> j_ik;
  j_ik.resize(num_joints);
  if (g_actual_js.position.size() >= num_joints)
  {
    for (int i = 0; i < num_joints ; i++)
    {
      j_ik[i] = g_actual_js.position[i]; 
      ROS_INFO( "j_ik[%d]: %0.2f [%0.2f %0.2f]", i, j_ik[i]*180.0/M_PI, q_min.data[i]*180.0/M_PI, q_max.data[i]*180.0/M_PI );
    }
  }

  try
  {
    g_tf_listener->lookupTransform(root_name, tip_name,
                                   ros::Time(0), t);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  

  tf::StampedTransform t_target_in_torso(g_target_origin * t, ros::Time::now(),
                                         "world", "ik_target");
/*
  geometry_msgs::TransformStamped target_trans_msg;
  tf::transformStampedTFToMsg(t_target_in_torso, target_trans_msg);
  g_tf_broadcaster->sendTransform(target_trans_msg);

  tf::StampedTransform t_target_origin(g_target_origin, ros::Time::now(),
                                         root_name, "target_origin");
  geometry_msgs::TransformStamped target_origin_msg;
  tf::transformStampedTFToMsg(t_target_origin, target_origin_msg);
  g_tf_broadcaster->sendTransform(target_origin_msg);
*/
  tf::transformStampedMsgToTF( t_msg, t );

  ik_tool(t, j_ik);
  g_js.header.stamp = ros::Time::now();
  g_js.position = j_ik;
  g_joint_pub->publish(g_js);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ik_test");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");

  root_name = "bandit_torso_link";
  tip_name = "left_hand_link";

  /* DFS: code to get model from parameter server */
  urdf::Model robot_model;
  std::string robot_desc;
  KDL::Chain chain;

  while(!loadRobotModel(n_private,robot_model,root_name,tip_name,robot_desc) && n.ok())
  {
    ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(robot_desc, tree))
  {
    ROS_ERROR("failed to extract kdl tree from xml robot description");
    return 1;
  }
  if (!tree.getNrOfSegments())
  {
    ROS_ERROR("empty tree. sad.");
    return 1;
  }

  if(!getKDLChain(robot_desc,root_name,tip_name,chain))
  {
    ROS_ERROR("Could not load kdl tree");
  }

  ROS_INFO("parsed tree successfully");
  ///////////////////////////////////////////////////////////////////////

  geometry_msgs::TransformStamped world_trans;
  world_trans.header.frame_id = "world";
  world_trans.child_frame_id = root_name;
  world_trans.transform.translation.x = 0;
  world_trans.transform.translation.y = 0;
  world_trans.transform.translation.z = 0;
  world_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

  // get joint maxs and mins
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
  while(link && link->name != root_name)
  {
    boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(link->parent_joint->name);
    ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
    if(!joint)
    {
      ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
      return false;
    }
    if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      num_joints++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }

  q_min.resize(num_joints);
  q_max.resize(num_joints);
  g_js.name.resize(num_joints);
  g_js.position.resize(num_joints);
  g_actual_js.name.resize(num_joints);
  g_actual_js.position.resize(num_joints);
  KDL::TreeFkSolverPosFull_recursive fk_solver(tree);
  g_fk_solver = &fk_solver;
  KDL::SegmentMap::const_iterator root_seg = tree.getRootSegment();
  string tree_root_name = root_seg->first;
  ROS_INFO("root: %s", tree_root_name.c_str());
  KDL::ChainFkSolverPos_recursive fk_solver_chain(chain);
  KDL::ChainIkSolverVel_pinv ik_solver_vel(chain);


  link = robot_model.getLink(tip_name);
  int i = 0;
  while(link && i < num_joints)
  {
    boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(link->parent_joint->name);
    ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );
    if(!joint)
    {
      ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
      return false;
    }
    if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      if( joint->type != urdf::Joint::CONTINUOUS )
      {
        g_js.name[num_joints-i-1] = joint->name;
        q_min.data[num_joints-i-1] = joint->safety->soft_lower_limit;
        q_max.data[num_joints-i-1] = joint->safety->soft_upper_limit;
      }
      else
      {
        g_js.name[num_joints-i-1] = joint->name;
        q_min.data[num_joints-i-1] = -M_PI;
        q_max.data[num_joints-i-1] = M_PI;
      }
      i++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }

  KDL::ChainIkSolverPos_NR_JL ik_solver_pos(chain, q_min, q_max,
                                            fk_solver_chain, ik_solver_vel, 
                                            100, 1e-6);
  g_ik_solver = &ik_solver_pos;
  ros::Subscriber target_sub = n.subscribe("target_frame", 1, target_cb);
  ros::Subscriber joint_sub = n.subscribe("joint_states", 1, joint_cb);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("target_joints", 1);
  g_joint_pub = &joint_pub; // ugly ugly
  tf::TransformBroadcaster tf_broadcaster;
  tf::TransformListener tf_listener;
  g_tf_broadcaster = &tf_broadcaster;
  g_tf_listener = &tf_listener;
  ros::Rate loop_rate(20);

  // set origin to be something we can comfortably reach
  g_target_origin = btTransform(btQuaternion::getIdentity(), btVector3(0, 0.1, 0));

  while (ros::ok())
  {
    world_trans.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(world_trans);

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}

