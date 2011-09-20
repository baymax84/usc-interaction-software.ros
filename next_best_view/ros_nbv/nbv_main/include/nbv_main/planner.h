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

#include <boost/random.hpp>


//#include <ros/ros.h>

#include <nbv_main/tree.h>
#include <nbv_main/voxel_grid.h>

class Planner
{
public:
  Planner(){};
  Planner(VoxelGrid*& voxel_grid, bool greedy);
  Planner(VoxelGrid*& voxel_grid, bool greedy, int depth = 3);

  bool plan(const Eigen::Vector3f& P0,
            std::vector<Eigen::Vector3f>& sampling_positions);
  bool plan_greedy(const Eigen::Vector3f& P0,
                   std::vector<Eigen::Vector3f>& sampling_positions);
  bool plan_expectation(const Eigen::Vector3f& P0,
                        std::vector<Eigen::Vector3f>& sampling_positions);
  bool plan_simple(const Eigen::Vector3f& P0,
                   std::vector<Eigen::Vector3f>& sampling_positions);
  bool collect_scans(const Eigen::Vector3f& P0,
                    std::vector<Eigen::Vector3f>& sampling_positions);


  bool plan_simple_FOV(const Eigen::Vector3f& P0,
                       std::vector<Eigen::Vector3f>& sampling_positions,
                       std::vector<std::vector<float> > &sampling_angle,
                       float direction);
  bool plan_expectation_FOV(const Eigen::Vector3f& P0,
                            std::vector<Eigen::Vector3f>& sampling_positions,
                            std::vector<std::vector<float> > &sampling_angle,
                            float direction);
  bool random_FOV(const Eigen::Vector3f& P0,
                  std::vector<Eigen::Vector3f>& sampling_positions,
                  std::vector<std::vector<float> > &sampling_angle,
                  float direction);

  Eigen::Vector3f getNewScanPosition(){return new_scan_position_;};
  void setFullSamplingPositions(std::vector<Eigen::Vector3f>& fsp)
    {full_sampling_positions_=fsp;};

  int calculateTravelCost(const Eigen::Vector3f& P0, const Eigen::Vector3f& P1);
  int computeClosestPosistion(const Eigen::Vector3f& P);
  int computeClosestPosistion(std::vector<Eigen::Vector3f>& sampling_positions,
                                       const Eigen::Vector3f& P);


  float getNewDirection(){return direction_;};
private:
  VoxelGrid* voxel_grid_;
  bool greedy_;
  int depth_;
  boost::mt19937 gen_;

  std::vector<Eigen::Vector3f> full_sampling_positions_;

  Eigen::Vector3f new_scan_position_;

  std::vector<Tree::node*> shortest_path_;

  float direction_;

};



#endif /* PLANNER_H_ */
