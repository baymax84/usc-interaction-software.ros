/*
 * tree.h
 *
 *  Created on: Dec 19, 2010
 *      Author: Christian Potthast
 */

#ifndef TREE_H_
#define TREE_H_

#include <vector>
#include <iostream>
#include <nbv_main/voxel_grid.h>


class Tree
{
public:
  typedef struct node {
     node* parent;
     std::vector<node*> child;
     int depth;
     int max_depth;
     float value;
     float cummultative_value;
     int sample_position;
     float direction;
     int travel_cost;
     int n_uv;
     VoxelGrid::array_type* voxel_grid;
     std::vector<Eigen::Vector3f> sampling_positions;
     Eigen::Vector3f scan_position;
  } node;


  Tree();
  void newChild(node* parent, int uv, int cost, VoxelGrid::array_type* vgrid, int index);
  void newChild(node* parent, int uv, int cost, VoxelGrid::array_type* vgrid, int index, float direction);
  void newChild(node* parent, double expectation, int cost, VoxelGrid::array_type* vgrid, int index);
  void newChild(node* parent, double expectation, int cost, VoxelGrid::array_type* vgrid, int index, float direction);

  // get all nodes on a specific depth
  std::vector<Tree::node*> getNodesOfDepth(const int depth);

  std::vector<Tree::node*> getshortestPath(const Eigen::Vector3f& P0);
  std::vector<Tree::node*> getPath();

  Tree::node* getNodesWithSmallestValue(std::vector<Tree::node*> nodes);

  // compute the next robot position by traversing the tree from down to top
  Tree::node* getNextBestNode();

  float computeNodeValue(int n_uv, int travel_cost);

  void setUVandCostWeight(int uv_w, int c_w){uv_weight_=uv_w;cost_weight_=c_w;};

  // clean up memory
  void cleanUp();
  // print the node
  void print(node* p_node);
  void print_short(node* p_node);
  // unit test for tree class
  void unitTest();

  node* getRoot(){return root_;};

private:
  node* root_ ;
  std::vector<std::vector<Tree::node*> > node_of_depth_;

  float uv_weight_;
  float cost_weight_;

};


#endif /* TREE_H_ */
