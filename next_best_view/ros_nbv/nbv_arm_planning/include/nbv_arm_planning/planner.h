/*
 * planner.h
 *
 *  Created on: Dec 19, 2010
 *      Author: Christian Potthast
 */

#ifndef PLANNER_H_
#define PLANNER_H_

#include <sstream>
#include <stdio.h>
#include <fstream>
#include <iomanip>


//#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>

#include <nbv_arm_planning/tree.h>
#include <nbv_arm_planning/voxel_grid.h>
#include <nbv_arm_planning/occupancy_grid.h>

#include <tf/transform_listener.h>

class Planner
{
public:
  Planner(){};
  Planner(VoxelGrid*& voxel_grid, bool greedy);
  Planner(VoxelGrid*& voxel_grid, bool greedy, int depth = 3);
  Planner(OccupancyGrid*& occupancy_grid, bool greedy, int depth = 3);


  bool collect_scans(const Eigen::Vector3f& P0,
                    std::vector<Eigen::Vector3f>& sampling_positions);


  bool plan_expectation_kinect(const Eigen::Vector3f& P0,
                               geometry_msgs::PoseArray& sampling_positions);


  geometry_msgs::Pose getNewScanPosition(){return new_scan_position_;};
  void setFullSamplingPositions(std::vector<Eigen::Vector3f>& fsp)
    {full_sampling_positions_=fsp;};

  int calculateTravelCost(const Eigen::Vector3f& P0, const Eigen::Vector3f& P1);
  int computeClosestPosistion(const Eigen::Vector3f& P);
  int computeClosestPosistion(std::vector<Eigen::Vector3f>& sampling_positions,
                                       const Eigen::Vector3f& P);


  float getNewDirection(){return direction_;};
private:
  //VoxelGrid* voxel_grid_;
  OccupancyGrid* occupancy_grid_;
  bool greedy_;
  int depth_;

  std::vector<Eigen::Vector3f> full_sampling_positions_;

  //Eigen::Vector3f new_scan_position_;
  geometry_msgs::Pose new_scan_position_;

  std::vector<Tree::node*> shortest_path_;

  float direction_;

};



#endif /* PLANNER_H_ */
