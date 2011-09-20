/*
 * graph.h
 *
 *  Created on: Apr 24, 2011
 *      Author: potthast
 */

#ifndef GRAPH_H_
#define GRAPH_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <vector>

class Graph
{
public:

  typedef struct node {

    std::vector<node*> edge;

  //  std::vector<node*> parent;
  //  std::vector<node*> child;
     geometry_msgs::Pose pose;
     node* prev;
     bool goal;
     bool start;
     float root_dist;
  } node;

  Graph(){};
  Graph(geometry_msgs::Pose pose);
  void insertNewNode(geometry_msgs::Pose pose);

  //void newChild(geometry_msgs::Pose pose);

  std::vector<Graph::node*> getKNearestNodes(geometry_msgs::Pose pose, float max_dist, int k);
  node* getRoot(){return root_;};
  node* getGoal(){return goal_;};

  void computeGoalDist(geometry_msgs::Pose pose);
  std::vector<geometry_msgs::PoseArray> getGoalPaths(std::vector<geometry_msgs::PoseArray> &paths,
                                                     geometry_msgs::Pose g_pose);

  float eulcideanDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);


  std::vector<geometry_msgs::PoseArray> getPaths(geometry_msgs::Pose g_pose);

  void print(Graph::node* node);
  void print(std::vector<geometry_msgs::PoseArray> &goal_paths);

  void test();

private:
  node* root_;
  node* goal_;
  std::vector<node*> nodes_;

};

#endif /* GRAPH_H_ */
