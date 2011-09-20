/*
 * planner.cpp
 *
 *  Created on: Dec 19, 2010
 *      Author: Christian Potthast
 */

#include <nbv_arm_planning/planner.h>

Planner::Planner(VoxelGrid*& voxel_grid, bool greedy)
{
 // voxel_grid_ = voxel_grid;
  greedy_ = greedy;
}

Planner::Planner(VoxelGrid*& voxel_grid, bool greedy, int depth)
{
 // voxel_grid_ = voxel_grid;
  greedy_ = greedy;
  depth_ = depth;
}

Planner::Planner(OccupancyGrid*& occupancy_grid, bool greedy, int depth)
{
  occupancy_grid_ = occupancy_grid;
  greedy_ = greedy;
  depth_ = depth;
}


bool Planner::plan_expectation_kinect(const Eigen::Vector3f& P0,
                                      geometry_msgs::PoseArray& sampling_poses)
{


  ROS_INFO("sampling poses: %d", (int)sampling_poses.poses.size());

  // Markov Random Field (MRV) stuff
  //occupancy_grid_->createMRFs();
  //occupancy_grid_->evalMrf();
  //occupancy_grid_->MRFToVG();
  //occupancy_grid_->mrfVisul();
  ROS_INFO("Planner - plan : MRF - DONE");

  // Ray traversal
  //occupancy_grid_->wall();
  OccupancyGrid::array_type grid = occupancy_grid_->getGrid();

 // ROS_INFO("!!!!!! plan begin: %f", grid[10][128][0].p);

  //std::vector<Eigen::Vector3i> a = occupancy_grid_->getUnknownVoxels(grid);
  std::vector<Eigen::Vector3i> a = occupancy_grid_->getUnknownVoxelsFOV(P0,grid, 0.0);



  ROS_INFO("BEFORE SIZE: %d", (int)a.size());
  occupancy_grid_->rayTraversal(P0, a, grid, true);
  occupancy_grid_->setGrid(grid);
  ROS_INFO("Planner - plan: Ray traversal - DONE");

  OccupancyGrid::array_type grid2 = occupancy_grid_->getGrid();

 // ROS_INFO("!!!!!! plan end: %f", grid2[10][128][0].p);

  //find volume in the space
  //std::vector< std::vector <Eigen::Vector3i> > volume;
  //occupancy_grid_->findVolumes(5);

  //std::vector<Eigen::Vector3i> new_scan_unknown_voxels = occupancy_grid_->getUnknownVoxels(grid);
  //ROS_INFO("NEW SCAN VOXEL SIZE: %d", (int)new_scan_unknown_voxels.size());

  double start = clock();

  //create the decision tree
  Tree search_tree;
  search_tree.setUVandCostWeight(1.0, 0.0);
  Tree::node* root = search_tree.getRoot();
  OccupancyGrid::array_type* vgrid = new OccupancyGrid::array_type;
  vgrid->resize(boost::extents[occupancy_grid_->dimensions_.z()][occupancy_grid_->dimensions_.y()][occupancy_grid_->dimensions_.x()]);
  *vgrid = occupancy_grid_->getGrid();
  root->voxel_grid = vgrid;
  //root->sampling_positions = sampling_positions;

  for(int depth=0; depth<depth_; depth++)
  {
    std::vector<Tree::node*> childs = search_tree.getNodesOfDepth(depth);
    int n_childs = childs.size();

    for(int i_child=0; i_child<n_childs; i_child++)
    {
      Tree::node* parent = childs[i_child];

     // ROS_INFO("size: %d", (int)parent->sampling_positions.size());
      for(unsigned int ii=0; ii<sampling_poses.poses.size(); ii++)
      {

        //get the occupancy grid
        OccupancyGrid::array_type* vgrid = new OccupancyGrid::array_type;
        vgrid->resize(boost::extents[occupancy_grid_->dimensions_.z()][occupancy_grid_->dimensions_.y()][occupancy_grid_->dimensions_.x()]);
        *vgrid = *parent->voxel_grid;

        //std::vector<Eigen::Vector3i> uvox = occupancy_grid_->getUnknownVoxels(*vgrid);

  //      const Eigen::Vector3f P = parent->sampling_positions[ii];
        double x = sampling_poses.poses[ii].position.x;
        double y = sampling_poses.poses[ii].position.y;
        double z = sampling_poses.poses[ii].position.z;
        const Eigen::Vector3f P(x , y, z);
        ROS_INFO("sample position index: %d", ii);
        //occupancy_grid_->estimateVoxelGrid(*vgrid, P, uvox);


        tf::Quaternion quat;
        quat.setX(sampling_poses.poses[ii].orientation.x);
        quat.setY(sampling_poses.poses[ii].orientation.y);
        quat.setZ(sampling_poses.poses[ii].orientation.z);
        quat.setW(sampling_poses.poses[ii].orientation.w);

        btMatrix3x3 rot(quat);

        btVector3 t(1,0,0);
        btVector3 r = rot * t;

        Eigen::Vector3f test;

        test.x() = r.x() + sampling_poses.poses[ii].position.x;
        test.y() = r.y() + sampling_poses.poses[ii].position.y;
        test.z() = r.z() + sampling_poses.poses[ii].position.z;

        occupancy_grid_->setViewDir(test);


        std::vector<Eigen::Vector3i> uvox = occupancy_grid_->getUnknownVoxelsFOV(P,*vgrid, 0.0);
        ROS_INFO("uvox size: %d", (int)uvox.size());

        occupancy_grid_->rayTraversal(P, uvox, *vgrid, true);
        //occupancy_grid_->rayTraversal(P, uvox, *vgrid, true);

        std::vector<Eigen::Vector3i> new_uvox = occupancy_grid_->getUnknownVoxels(*vgrid);
        ROS_INFO("new_uvox size: %d", (int)new_uvox.size());

        //store the data in the tree
 //       int cost = calculateTravelCost(P0, parent->sampling_positions[ii]);
        int cost = 0;
        double expectation = occupancy_grid_->computeExpectation(*vgrid);
        ROS_INFO("expectation: %f", expectation);
       // search_tree.newChild(parent, expectation, cost, vgrid, ii);

        search_tree.newChild(parent, (int)new_uvox.size(), cost, vgrid, ii);

       // search_tree.newChild(parent, (int)new_uvox.size(), cost, vgrid, ii);
      }
    }


   int max_depth = root->max_depth;
    std::vector<Tree::node*> s_nodes = search_tree.getNodesOfDepth(max_depth);
    Tree::node* n = search_tree.getNodesWithSmallestValue(s_nodes);
    search_tree.print_short(n);


    //Tree::node* n = s_nodes[5];

   // occupancy_grid_->setGrid(*n->voxel_grid);
    new_scan_position_ = sampling_poses.poses[n->sample_position];
    //new_scan_position_ = n->scan_position;
    //sampling_positions = n->sampling_positions;

    double finish = clock();
    std::cerr << "Planning DONE in: "<< ( (finish - start)/CLOCKS_PER_SEC ) << std::endl;


//    float expectation = (-1) * n->value;
//
//    std::ifstream ifile("/home/potthast/projects/data/result.txt");
//    std::ofstream file;
//    if((bool)ifile)
//    {
//      //open file
//      file.open("/home/potthast/projects/data/result.txt", std::ios::app);
//    }else{
//      //create a new file
//      file.open("/home/potthast/projects/data/result.txt");
//    }
//    file << (int)new_scan_unknown_voxels.size() << " " << std::fixed << std::setprecision(3) << (float)expectation << "\n";
//    file.close();
//
//    std::cout << "Writing to disk - DONE" << std::endl;



  search_tree.cleanUp();

  }


  //int index = computeClosestPosistion(full_sampling_positions_, new_scan_position_);
  //ROS_INFO("scan number: %d", index+1);



//  occupancy_grid_->colorUnknownVoxels(a);


  grid = occupancy_grid_->getGrid();
  a = occupancy_grid_->getUnknownVoxels(grid);

  float percent = occupancy_grid_->dimensions_.x() * occupancy_grid_->dimensions_.y() * occupancy_grid_->dimensions_.z();
  percent = (float)a.size() / percent;
  percent = percent * 100.0;

  ROS_INFO("Unknown voxels: %d ; percent: %f", (int)a.size(), percent);

  occupancy_grid_->voxelGridMsg();

  return true;
}





/////////////////////////////////////////////////////////////////////////////////
// Other stuff //
/////////////////////////////////////////////////////////////////////////////////

bool Planner::collect_scans(const Eigen::Vector3f& P0,
                          std::vector<Eigen::Vector3f>& sampling_positions)
{

  Eigen::Vector3f pos = sampling_positions[0];
//  new_scan_position_ = pos;

  sampling_positions.erase(sampling_positions.begin());

 // if(sampling_positions.size() < 1)
 //   return false;

  return true;
}


int Planner::calculateTravelCost(const Eigen::Vector3f& P0, const Eigen::Vector3f& P1)
{
  int n_pos = full_sampling_positions_.size();
  int cost = -1;

  int p0_index = computeClosestPosistion(P0);
  int p1_index = computeClosestPosistion(P1);

  if(p1_index <= p0_index+((int)floor(n_pos/2)))
    cost = p1_index-p0_index;
  else
    cost = p0_index+n_pos-p1_index;

  return cost;
}

int Planner::computeClosestPosistion(const Eigen::Vector3f& P)
{
  float dist = 100.0;
  int index = 0;

  for(unsigned int ii=0; ii<full_sampling_positions_.size(); ii++)
  {
    float tmp_dist = sqrt(pow(P.x() - full_sampling_positions_[ii].x(),2) +
                          pow(P.y() - full_sampling_positions_[ii].y(),2) +
                          pow(P.z() - full_sampling_positions_[ii].z(),2));
    if(tmp_dist<dist){
      dist = tmp_dist;
      index=ii;
    }
  }
  return index;
}

int Planner::computeClosestPosistion(std::vector<Eigen::Vector3f>& sampling_positions,
                                     const Eigen::Vector3f& P)
{
  float dist = 100.0;
  int index = 0;

  for(unsigned int ii=0; ii<sampling_positions.size(); ii++)
  {
    float tmp_dist = sqrt(pow(P.x() - sampling_positions[ii].x(),2) +
                          pow(P.y() - sampling_positions[ii].y(),2) +
                          pow(P.z() - sampling_positions[ii].z(),2));
    if(tmp_dist<dist){
      dist = tmp_dist;
      index=ii;
    }
  }
  return index;
}
