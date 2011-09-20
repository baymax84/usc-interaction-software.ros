/*
 * graph.cpp
 *
 *  Created on: Apr 24, 2011
 *      Author: potthast
 */

#include <nbv_arm_planning/graph.h>

Graph::Graph(geometry_msgs::Pose pose)
{
  root_ = new node;
  root_->pose = pose;
  root_->goal = false;
  root_->root_dist = (float)0.0;
  root_->prev=root_;
  nodes_.push_back(root_);
}

void Graph::insertNewNode(geometry_msgs::Pose pose)
{
  std::vector<Graph::node*> adjacentNodes;
  adjacentNodes = getKNearestNodes(pose, 0.5, 5);

  Graph::node* new_node = new node;
  //new_node->parent = parent;
  new_node->edge = adjacentNodes;
  new_node->pose = pose;
  new_node->goal = false;
  new_node->root_dist = eulcideanDistance(root_->pose, pose);

  nodes_.push_back(new_node);

  for(unsigned int ii=0; ii<adjacentNodes.size(); ii++)
  {
    adjacentNodes[ii]->edge.push_back(new_node);
  }
}

/*
void Graph::newChild(geometry_msgs::Pose pose)
{
  std::vector<Graph::node*> parent;
  parent = getKNearestNodes(pose, 0.4, 5);


  Graph::node* child = new node;
  child->parent = parent;
  child->pose = pose;
  child->goal = false;
  child->index = 0;

  nodes_.push_back(child);
  for(unsigned int ii=0; ii<parent.size(); ii++)
  {
    parent[ii]->child.push_back(child);
  }
}
*/


std::vector<Graph::node*> Graph::getKNearestNodes(geometry_msgs::Pose pose, float max_dist, int k)
{
  std::vector<Graph::node*> k_nearest_nodes;

  float dist=0.0;
  for(unsigned int ii=0; ii<nodes_.size(); ii++)
  {
    //if(k_nearest_nodes.size() == 10)
    //  break;

    dist=eulcideanDistance(pose, nodes_[ii]->pose);

    if(dist < max_dist)
      k_nearest_nodes.push_back(nodes_[ii]);
  }

  return k_nearest_nodes;
}

float Graph::eulcideanDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  float dist;

  dist = sqrt( pow(p1.position.x - p2.position.x, 2) +
               pow(p1.position.y - p2.position.y, 2) +
               pow(p1.position.z - p2.position.z, 2) );

  return dist;

}

void Graph::computeGoalDist(geometry_msgs::Pose pose)
{
  // update the goal distance
  for(unsigned int ii=0; ii<nodes_.size(); ii++)
  {
    nodes_[ii]->root_dist = eulcideanDistance(nodes_[ii]->pose, pose);
  }
}

std::vector<geometry_msgs::PoseArray> Graph::getGoalPaths(std::vector<geometry_msgs::PoseArray> &paths,
                                                          geometry_msgs::Pose g_pose)
{
  std::vector<geometry_msgs::PoseArray> goal_paths;

  for(unsigned int i=0; i<paths.size(); i++)
  {
    geometry_msgs::Pose pose = paths[i].poses[paths[i].poses.size()-1];
    if( pose.position.x == g_pose.position.x &&
        pose.position.y == g_pose.position.y &&
        pose.position.z == g_pose.position.z)
    {
      goal_paths.push_back(paths[i]);
    }
  }

  return goal_paths;
}

std::vector<geometry_msgs::PoseArray> Graph::getPaths(geometry_msgs::Pose g_pose)
{
  std::vector<geometry_msgs::PoseArray> paths;
  std::vector<std::vector<Graph::node*> > paths_nodes;
  bool done = false;

  Graph::node* node = root_;
  std::vector<Graph::node*> node_path;
  node_path.push_back(node);
  paths_nodes.push_back(node_path);
  for(unsigned int i=1; i<4; i++)
  {
    done = true;
    //ROS_INFO("TEST-1: %d", (int)paths_nodes.size());
    unsigned int pn_size = paths_nodes.size();
    for(unsigned int j=0; j<pn_size; j++)
    {
      // find the nodes which we want to extend
      if(i == paths_nodes[j].size())
      {
        done = false;
        node = paths_nodes[j][paths_nodes[j].size()-1];
      //  ROS_INFO("TEST-2: %d", (int)node->edge.size());
        for(unsigned int i_edge=0; i_edge<node->edge.size(); i_edge++)
        {
          std::vector<Graph::node*> new_node_path(paths_nodes[j]);

          // check if node is already visited in the path
          bool exist = false;
          for(unsigned int k=0; k<new_node_path.size(); k++)
          {
            geometry_msgs::Pose pose = new_node_path[k]->pose;
            if( pose.position.x == node->edge[i_edge]->pose.position.x &&
                pose.position.y == node->edge[i_edge]->pose.position.y &&
                pose.position.z == node->edge[i_edge]->pose.position.z)
            {
              exist = true;
              break;
            }
          }

          // check if the extended node is increasing the distance to the root node
          if(exist == false && node->root_dist < node->edge[i_edge]->root_dist)
          {
           // print(node);
           // print(node->edge[i_edge]);
            exist = true;
          }

          if(!exist)
          {
            new_node_path.push_back(node->edge[i_edge]);
            paths_nodes.push_back(new_node_path);
          }
        }
      }
    }
    if(done)
      break;




    if(i>2){
    for(unsigned int j=0; j<paths_nodes.size(); j++)
    {
      if(i-1 == paths_nodes[j].size())
      {
        geometry_msgs::Pose pose = paths_nodes[j][paths_nodes[j].size()-1]->pose;
        if( pose.position.x != g_pose.position.x ||
            pose.position.y != g_pose.position.y ||
            pose.position.z != g_pose.position.z)
        {
          paths_nodes[j].clear();
          paths_nodes.erase(paths_nodes.begin() + j);
          j--;
        }
      }
    }
    }



  }




  for(unsigned int i=0; i<paths_nodes.size(); i++)
  {
    geometry_msgs::PoseArray waypoints;
    for(unsigned int j=0; j<paths_nodes[i].size(); j++)
    {
      waypoints.poses.push_back(paths_nodes[i][j]->pose);
    }
    paths.push_back(waypoints);
  }

  return paths;
}

void Graph::print(Graph::node* node)
{

  std::cout << "n_eges: " << node->edge.size() << std::endl;
  std::cout << "position: " << node->pose.position.x << " " << node->pose.position.y << " " << node->pose.position.z << std::endl;
  std::cout << "root_dist: " << node->root_dist << std::endl;

  for(unsigned int ii=0; ii<node->edge.size(); ii++)
  {
    std::cout << "position: " << node->edge[ii]->pose.position.x << " "
        << node->edge[ii]->pose.position.y << " "
        << node->edge[ii]->pose.position.z << std::endl;
  }

  std::cout << "--------------------------" << std::endl;

}

void Graph::print(std::vector<geometry_msgs::PoseArray> &goal_paths)
{
  std::cout << "number of goal paths: " << goal_paths.size() << std::endl;
  std::cout << "------" << std::endl;

  for(unsigned int i=0; i<goal_paths.size(); i++)
  {
    std::cout << "number of node in path: " << goal_paths[i].poses.size() << std::endl;
    for(unsigned int j=0; j<goal_paths[i].poses.size(); j++)
    {
      geometry_msgs::Pose pose = goal_paths[i].poses[j];
      std::cout << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
    }
    std::cout << "--------------------------" << std::endl;
  }
}

void Graph::test()
{
  geometry_msgs::Pose pose;
  pose.position.x = 0.2;
  pose.position.y = 0.4;
  pose.position.z = 0.0;
  insertNewNode(pose);
  pose.position.x = 0.0;
  pose.position.y = -0.5;
  pose.position.z = 0.0;
  insertNewNode(pose);
  pose.position.x = 0.0;
  pose.position.y = 0.5;
  pose.position.z = 0.0;
  insertNewNode(pose);
  pose.position.x = 0.59;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  insertNewNode(pose);
  pose.position.x = 0.0;
  pose.position.y = 0.8;
  pose.position.z = 0.0;
  insertNewNode(pose);
  pose.position.x = 0.0;
  pose.position.y = -0.8;
  pose.position.z = 0.0;
  insertNewNode(pose);
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  insertNewNode(pose);
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;

 // findAndSetGoal(pose);
}


