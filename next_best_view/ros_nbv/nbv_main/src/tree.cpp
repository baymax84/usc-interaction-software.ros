/*
 * tree.cpp
 *
 *  Created on: Dec 19, 2010
 *      Author: Christian Potthast
 */

#include <nbv_main/tree.h>

Tree::Tree()
{
  root_ = new node;
  root_->value = 0;
  root_->parent = NULL;
  root_->depth = 0;
  root_->max_depth = 0;
  root_->sample_position = -1;

  std::vector<Tree::node*> node_of_depth;
  node_of_depth.push_back(root_);
  node_of_depth_.push_back(node_of_depth);
}

void Tree::newChild(node* parent, int uv, int cost, VoxelGrid::array_type* vgrid, int index)
{
  node* child = new node;
  child->parent = parent;
  child->n_uv = uv;
  child->travel_cost = cost + parent->travel_cost;
  child->value = computeNodeValue(uv, cost);
  child->depth = parent->depth + 1;
  child->voxel_grid = vgrid;
  child->sample_position = index;
  child->sampling_positions = parent->sampling_positions;
  child->scan_position = child->sampling_positions[child->sample_position];
  if(child->sampling_positions.size() > 0)
    child->sampling_positions.erase(child->sampling_positions.begin() + index);

  if(child->depth > (int)node_of_depth_.size()-1)
  {
    std::vector<Tree::node*> node_of_depth;
    node_of_depth.push_back(child);
    node_of_depth_.push_back(node_of_depth);
  }else{
    node_of_depth_[child->depth].push_back(child);
  }

  if(root_->max_depth < child->depth)
    root_->max_depth++;

  parent->child.push_back(child);

}

void Tree::newChild(node* parent, int uv, int cost, VoxelGrid::array_type* vgrid, int index, float direction)
{
  node* child = new node;
  child->parent = parent;
  child->n_uv = uv;
  child->travel_cost = cost + parent->travel_cost;
  child->value = computeNodeValue(uv, cost);
  child->depth = parent->depth + 1;
  child->voxel_grid = vgrid;
  child->sample_position = index;
  child->direction = direction;
  child->sampling_positions = parent->sampling_positions;
  child->scan_position = child->sampling_positions[child->sample_position];
  if(child->sampling_positions.size() > 0)
    child->sampling_positions.erase(child->sampling_positions.begin() + index);

  if(child->depth > (int)node_of_depth_.size()-1)
  {
    std::vector<Tree::node*> node_of_depth;
    node_of_depth.push_back(child);
    node_of_depth_.push_back(node_of_depth);
  }else{
    node_of_depth_[child->depth].push_back(child);
  }

  if(root_->max_depth < child->depth)
    root_->max_depth++;

  parent->child.push_back(child);

}



void Tree::newChild(node* parent, double expectation, int cost, VoxelGrid::array_type* vgrid, int index)
{
  node* child = new node;
  child->parent = parent;
  child->travel_cost = cost + parent->travel_cost;
  child->value = expectation;
  child->depth = parent->depth + 1;
  child->voxel_grid = vgrid;
  child->sample_position = index;
  child->sampling_positions = parent->sampling_positions;
  child->scan_position = child->sampling_positions[child->sample_position];
  if(child->sampling_positions.size() > 0)
    child->sampling_positions.erase(child->sampling_positions.begin() + index);

  if(child->depth > (int)node_of_depth_.size()-1)
  {
    std::vector<Tree::node*> node_of_depth;
    node_of_depth.push_back(child);
    node_of_depth_.push_back(node_of_depth);
  }else{
    node_of_depth_[child->depth].push_back(child);
  }

  if(root_->max_depth < child->depth)
    root_->max_depth++;

  parent->child.push_back(child);

}

void Tree::newChild(node* parent, double expectation, int cost, VoxelGrid::array_type* vgrid, int index, float direction)
{
  node* child = new node;
  child->parent = parent;
  child->travel_cost = cost + parent->travel_cost;
  child->value = expectation;
  child->depth = parent->depth + 1;
  child->voxel_grid = vgrid;
  child->sample_position = index;
  child->direction = direction;
  child->sampling_positions = parent->sampling_positions;
  child->scan_position = child->sampling_positions[child->sample_position];
  if(child->sampling_positions.size() > 0)
    child->sampling_positions.erase(child->sampling_positions.begin() + index);

  if(child->depth > (int)node_of_depth_.size()-1)
  {
    std::vector<Tree::node*> node_of_depth;
    node_of_depth.push_back(child);
    node_of_depth_.push_back(node_of_depth);
  }else{
    node_of_depth_[child->depth].push_back(child);
  }

  if(root_->max_depth < child->depth)
    root_->max_depth++;

  parent->child.push_back(child);

}



std::vector<Tree::node*> Tree::getNodesOfDepth(const int depth)
{
  return node_of_depth_[depth];
}

void Tree::cleanUp()
{
  for(int ii=node_of_depth_.size()-1; ii>0; ii--)
  {
    for(unsigned int jj=0; jj<node_of_depth_[ii].size(); jj++)
    {
      delete node_of_depth_[ii][jj]->voxel_grid;
      delete node_of_depth_[ii][jj];
    }
  }
  delete root_;
}

std::vector<Tree::node*> Tree::getPath()
{
  std::vector<Tree::node*> path;

  int max_depth = root_->max_depth;
  std::vector<Tree::node*> nodes = getNodesOfDepth(max_depth);
  node* n = getNodesWithSmallestValue(nodes);

  Tree::node* last_node = n;
  while(last_node->depth > 0)
  {
    path.push_back(last_node);
    last_node = last_node->parent;
  }

  return path;
}


std::vector<Tree::node*> Tree::getshortestPath(const Eigen::Vector3f& P0)
{
  Eigen::Vector3f p0 = P0;
  std::vector<Tree::node*> shortest_path;

  std::vector<Tree::node*> path;
  path = getPath();

  unsigned int path_size = (int)path.size();



  while(shortest_path.size() != path_size)
  {
    float tmp_dist = 1000.0;
    int index = -1;
    for(int ii=0; ii<(int)path.size(); ii++)
    {
      const Eigen::Vector3f p1(path[ii]->scan_position);
      float dist = sqrt( pow(p1.x() - p0.x(),2) +  pow(p1.y() - p0.y(),2) +  pow(p1.z() - p0.z(),2) );

      if(dist < tmp_dist)
      {
        tmp_dist = dist;
        index = ii;
      }
    }

    p0 = path[index]->scan_position;
    shortest_path.push_back(path[index]);
    path.erase(path.begin() + index);

  }

  return shortest_path;
}


Tree::node* Tree::getNodesWithSmallestValue(std::vector<Tree::node*> nodes)
{
  node* n = nodes[0];
  float tmp = nodes[0]->value;
  // find the node with the lowest value
  for(unsigned int ii=0; ii<nodes.size(); ii++)
  {
    if( nodes[ii]->value < tmp )
    {
      tmp = nodes[ii]->value;
      n = nodes[ii];
    }
  }
  return n;
}


Tree::node* Tree::getNextBestNode()
{
  Eigen::Vector3f new_scan_position;

  int max_depth = root_->max_depth;
  std::vector<Tree::node*> nodes = getNodesOfDepth(max_depth);
  node* n = getNodesWithSmallestValue(nodes);

  Tree::node* last_node = n;
  while(last_node->depth > 0)
  {
    last_node = last_node->parent;
  }

  return last_node;
}

float Tree::computeNodeValue(int n_uv, int travel_cost)
{
  float node_value;
  node_value = (float)n_uv*uv_weight_ + float(travel_cost)*cost_weight_;
  return node_value;
}


void Tree::print(node* p_node)
{
  std::cout << "value: " << p_node->value << std::endl;
  std::cout << "max depth: " << p_node->max_depth << std::endl;
  std::cout << "depth: " << p_node->depth << std::endl;
  std::cout << "number of child's: " << p_node->child.size() << std::endl;

  std::cout << "child values:";
  for(unsigned int ii=0; ii<p_node->child.size(); ii++)
  {
    std::cout << " " << p_node->child[ii]->value;
  }
  std::cout << std::endl;

}

void Tree::print_short(node* p_node)
{
  std::cout << "d: " << p_node->depth;
  std::cout << " | index: " << p_node->sample_position << std::endl;
  std::cout << " | v: " << p_node->value << std::endl;
  std::cout << " ";
  for(unsigned int ii=0; ii<p_node->child.size(); ii++)
  {
    std::cout << " | " << p_node->child[ii]->value;
  }
  std::cout << std::endl;
}


void Tree::unitTest()
{
  std::vector<Eigen::Vector3f> tmp;
  tmp.push_back(Eigen::Vector3f(0.0,0.0,0.0));
  tmp.push_back(Eigen::Vector3f(1.0,0.0,0.0));
  tmp.push_back(Eigen::Vector3f(0.0,1.0,0.0));
  tmp.push_back(Eigen::Vector3f(0.0,0.0,1.0));
  tmp.push_back(Eigen::Vector3f(1.0,0.0,1.0));
  tmp.push_back(Eigen::Vector3f(1.0,1.0,1.0));
  tmp.push_back(Eigen::Vector3f(0.5,0.0,0.0));
  tmp.push_back(Eigen::Vector3f(0.0,0.5,0.0));
  tmp.push_back(Eigen::Vector3f(0.0,0.0,0.5));
  tmp.push_back(Eigen::Vector3f(0.5,0.0,0.5));
  root_->sampling_positions = tmp;

  VoxelGrid::array_type* vgrid_1 = new VoxelGrid::array_type;
  newChild(root_, 1.0, 1, vgrid_1, 1);
  VoxelGrid::array_type* vgrid_2 = new VoxelGrid::array_type;
  newChild(root_, 2.0, 1, vgrid_2, 2);
  VoxelGrid::array_type* vgrid_3 = new VoxelGrid::array_type;
  newChild(root_, 3.0, 1, vgrid_3, 3);

  node* child_1 = root_->child[0];
  VoxelGrid::array_type* vgrid_4 = new VoxelGrid::array_type;
  newChild(child_1, 1.1, 1, vgrid_4, 4);
  VoxelGrid::array_type* vgrid_5 = new VoxelGrid::array_type;
  newChild(child_1, 1.2, 1, vgrid_5, 5);

  node* child_2 = root_->child[1];
  VoxelGrid::array_type* vgrid_6 = new VoxelGrid::array_type;
  newChild(child_2, 2.1, 1, vgrid_6, 6);
  VoxelGrid::array_type* vgrid_7 = new VoxelGrid::array_type;
  newChild(child_2, 2.2, 1, vgrid_7, 7);

  node* child_3 = root_->child[2];
  VoxelGrid::array_type* vgrid_8 = new VoxelGrid::array_type;
  newChild(child_3, 3.1, 1, vgrid_8, 8);
  VoxelGrid::array_type* vgrid_9 = new VoxelGrid::array_type;
  newChild(child_3, 3.2, 1, vgrid_9, 9);

  std::vector<Tree::node*> nodes = getNodesOfDepth(0);
  if(nodes.size() == 1)
    std::cout << "OK - number of nodes of level 0: 1=" << nodes.size() << std::endl;
  else
  {
    std::cout << "ERROR - number of nodes of level 0: 1=" << nodes.size();
    std::cout << std::endl;
    std::cout << "child values:";
    for(unsigned int ii=0; ii<nodes.size(); ii++)
    {
      std::cout << " " << nodes[ii]->value;
    }
    std::cout << std::endl;
  }

  if(nodes[0]->max_depth == 2)
    std::cout << "OK - max depth 0: 2=" << nodes[0]->max_depth << std::endl;
  else
  {
    std::cout << "ERROR - max depth 0: 2=" << nodes[0]->max_depth << std::endl;
  }

  nodes = getNodesOfDepth(1);
  if(nodes.size() == 3)
    std::cout << "OK - number of nodes of level 1: 3=" << nodes.size() << std::endl;
  else
  {
    std::cout << "ERROR - number of nodes of level 1: 3=" << nodes.size();
    std::cout << std::endl;
    std::cout << "child values:";
    for(unsigned int ii=0; ii<nodes.size(); ii++)
    {
      std::cout << " " << nodes[ii]->value;
    }
    std::cout << std::endl;
  }

  node* n = getNodesWithSmallestValue(nodes);
  if (fabs(n->value - 1.0) < 0.00001)
    std::cout << "OK - smallest node value depth 1: 1.0=" << n->value << std::endl;
  else
  {
    std::cout << "ERROR - smallest node value depth 1: 1.0=" << n->value << std::endl;
  }


  nodes = getNodesOfDepth(2);
  if(nodes.size() == 6)
    std::cout << "OK - number of nodes of level 1: 6=" << nodes.size() << std::endl;
  else
  {
    std::cout << "ERROR - number of nodes of level 1: 6=" << nodes.size();
    std::cout << std::endl;
    std::cout << "child values:";
    for(unsigned int ii=0; ii<nodes.size(); ii++)
    {
      std::cout << " " << nodes[ii]->value;
    }
    std::cout << std::endl;
  }

  n = getNodesWithSmallestValue(nodes);
  if (fabs(n->value - 1.1) < 0.00001)
    std::cout << "OK - smallest node value depth 2: 1.1=" << n->value << std::endl;
  else
  {
    std::cout << "ERROR - smallest node value depth 2: 1.1=" << n->value << std::endl;
  }

  n = getNextBestNode();
  Eigen::Vector3f new_scan_position = n->scan_position;
  if(new_scan_position.x() == 1 && new_scan_position.y() == 0 && new_scan_position.z() == 0)
  {
    std::cout << "OK - next scan position is 100="
      << new_scan_position.x() << new_scan_position.y() << new_scan_position.z() << std::endl;
  }else{
    std::cout << "ERROR - next scan position is 100=: "
          << new_scan_position.x() << new_scan_position.y() << new_scan_position.z() << std::endl;
  }

  cleanUp();

}

