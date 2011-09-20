/*
 * planner.cpp
 *
 *  Created on: Dec 19, 2010
 *      Author: Christian Potthast
 */

#include <nbv_main/planner.h>

Planner::Planner(VoxelGrid*& voxel_grid, bool greedy)
{
  voxel_grid_ = voxel_grid;
  greedy_ = greedy;
}

Planner::Planner(VoxelGrid*& voxel_grid, bool greedy, int depth)
{
  voxel_grid_ = voxel_grid;
  greedy_ = greedy;
  depth_ = depth;

  boost::mt19937 gen(std::time(NULL));
  gen_ = gen;

}

bool Planner::plan(const Eigen::Vector3f& P0,
                   std::vector<Eigen::Vector3f>& sampling_positions)
{
  // Markov Random Field (MRV) stuff
  voxel_grid_->createMRFs();
  voxel_grid_->evalMrf();
  voxel_grid_->MRFToVG();
  voxel_grid_->mrfVisul();
  ROS_INFO("Planner - plan : MRF - DONE");

  // Ray traversal
  VoxelGrid::array_type grid = voxel_grid_->getVoxelGrid();
  VoxelGrid::array_type start_grid_ = voxel_grid_->getVoxelGrid();
  std::vector<Eigen::Vector3i> a = voxel_grid_->getUnknownVoxels(grid);
  voxel_grid_->rayTraversal2(P0, a, grid, true);
  voxel_grid_->setVoxelGrid(grid);
  ROS_INFO("Planner - plan: Ray traversal - DONE");

  // greedy means: plan step by step and always take the next
  // position which minimizes the most unknown voxels
  if(greedy_)
  {
    // get the new scan position
    new_scan_position_ = voxel_grid_->findNBV();
  }else{
    double start = clock();

    //create the decision tree
    Tree search_tree;
    search_tree.setUVandCostWeight(1.0, 1000);
    Tree::node* root = search_tree.getRoot();
    VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
    vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
    *vgrid = voxel_grid_->getVoxelGrid();
    root->voxel_grid = vgrid;
    root->sampling_positions = sampling_positions;

    for(int depth=0; depth<depth_; depth++)
    {
      std::vector<Tree::node*> childs = search_tree.getNodesOfDepth(depth);
      int n_childs = childs.size();

      for(int i_child=0; i_child<n_childs; i_child++)
      {
        Tree::node* parent = childs[i_child];

        ROS_INFO("size: %d", (int)parent->sampling_positions.size());
        for(unsigned int ii=0; ii<parent->sampling_positions.size(); ii++)
        {
          //get the voxel grid
          VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
          vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
          *vgrid = *parent->voxel_grid;

          std::vector<Eigen::Vector3i> uvox = voxel_grid_->getUnknownVoxels(*vgrid);
          ROS_INFO("uvox size: %d", (int)uvox.size());
          const Eigen::Vector3f P = parent->sampling_positions[ii];
          voxel_grid_->estimateVoxelGrid(*vgrid, P, uvox);
          std::vector<Eigen::Vector3i> new_uvox = voxel_grid_->getUnknownVoxels(*vgrid);
          ROS_INFO("new_uvox size: %d", (int)new_uvox.size());

          //store the data in the tree
          int cost = calculateTravelCost(P0, parent->sampling_positions[ii]);
          search_tree.newChild(parent, (int)new_uvox.size(), cost, vgrid, ii);
        }

      }

    }


    // TEST //
    std::cout << "-- TEST -- " << std::endl;
    /*
    VoxelGrid::array_type vgrid_0 = voxel_grid_->getVoxelGrid();
    const Eigen::Vector3f P_0 = full_sampling_positions_[1];
    std::vector<Eigen::Vector3i> uvox_0 = voxel_grid_->getUnknownVoxels(vgrid_0);
    voxel_grid_->estimateVoxelGrid(vgrid_0, P_0, uvox_0);
    std::vector<Eigen::Vector3i> uvox_new_0 = voxel_grid_->getUnknownVoxels(vgrid_0);

    VoxelGrid::array_type vgrid_2 = voxel_grid_->getVoxelGrid();
    const Eigen::Vector3f P_2 = full_sampling_positions_[9];
    std::vector<Eigen::Vector3i> uvox_2 = voxel_grid_->getUnknownVoxels(vgrid_2);
    voxel_grid_->estimateVoxelGrid(vgrid_2, P_2, uvox_2);
    std::vector<Eigen::Vector3i> uvox_new_2 = voxel_grid_->getUnknownVoxels(vgrid_2);

    VoxelGrid::array_type vgrid_0_2 = vgrid_0;
    const Eigen::Vector3f P_0_2 = full_sampling_positions_[9];
    std::vector<Eigen::Vector3i> uvox_0_2 = voxel_grid_->getUnknownVoxels(vgrid_0_2);
    voxel_grid_->estimateVoxelGrid(vgrid_0_2, P_0_2, uvox_0_2);
    std::vector<Eigen::Vector3i> uvox_new_0_2 = voxel_grid_->getUnknownVoxels(vgrid_0_2);

    VoxelGrid::array_type vgrid_2_0 = vgrid_2;
    const Eigen::Vector3f P_2_0 = full_sampling_positions_[1];
    std::vector<Eigen::Vector3i> uvox_2_0 = voxel_grid_->getUnknownVoxels(vgrid_2_0);
    voxel_grid_->estimateVoxelGrid(vgrid_2_0, P_2_0, uvox_2_0);
    std::vector<Eigen::Vector3i> uvox_new_2_0 = voxel_grid_->getUnknownVoxels(vgrid_2_0);
*/


    //const Eigen::Vector3f P_0 = full_sampling_positions_[0];
    const Eigen::Vector3f P_1 = sampling_positions[5];

    /*
    voxel_grid_->rayTraversal(P_0);
    voxel_grid_->rayTraversal(P_1);
    VoxelGrid::array_type after_vg = voxel_grid_->getVoxelGrid();
    std::vector<Eigen::Vector3i> after_uvox_new_2_0 = voxel_grid_->getUnknownVoxels(after_vg);
    std::cout << "after_uvox_new_2_0: " << after_uvox_new_2_0.size()  << std::endl;
    */


    //VoxelGrid::array_type start_grid_2 = start_grid_;
    //voxel_grid_->setVoxelGrid(start_grid_);
 /*
    std::vector<Eigen::Vector3i> a = voxel_grid_->getUnknownVoxels(start_grid_);
    std::cout << "a: " << a.size()  << std::endl;
  //  std::cout << "a: " << a.size()  << std::endl;
    voxel_grid_->rayTraversal2(P0,a,start_grid_);
    std::vector<Eigen::Vector3i> b = voxel_grid_->getUnknownVoxels(start_grid_);
    voxel_grid_->rayTraversal2(P_1, b,start_grid_);
    std::vector<Eigen::Vector3i> eeee = voxel_grid_->getUnknownVoxels(start_grid_);
*/


/*
    std::vector<Eigen::Vector3i> aa = voxel_grid_->getUnknownVoxels(start_grid_2);
    std::cout << "aa: " << aa.size()  << std::endl;
    voxel_grid_->rayTraversal2(P_0, aa,start_grid_2);
    std::vector<Eigen::Vector3i> bb = voxel_grid_->getUnknownVoxels(start_grid_2);
    voxel_grid_->rayTraversal2(P_1, bb, start_grid_2);
    std::vector<Eigen::Vector3i> eeee2 = voxel_grid_->getUnknownVoxels(start_grid_2);
*/





    //voxel_grid_->rayTraversal(P_0);
    //voxel_grid_->rayTraversal(P0);
    //VoxelGrid::array_type after_vg_2 = voxel_grid_->getVoxelGrid();
    //    std::vector<Eigen::Vector3i> after_2_uvox_new_2_0 = voxel_grid_->getUnknownVoxels(after_vg_2);







  //  std::cout << "P_0 - uvox_0: " << uvox_0.size() << " | uvox_new_0: " << uvox_new_0.size() << std::endl;
  //  std::cout << "P_2 - uvox_2: " << uvox_2.size() << " | uvox_new_2: " << uvox_new_2.size() << std::endl;
  //  std::cout << "P_0_2 - uvox_0_2: " << uvox_0_2.size() << " | uvox_new_0_2: " << uvox_new_0_2.size() << std::endl;
  //  std::cout << "P_2_0 - uvox_2_0: " << uvox_2_0.size() << " | uvox_new_2_0: " << uvox_new_2_0.size() << std::endl;

   //  std::cout << "after_uvox_new_2_0: " << after_uvox_new_2_0.size()  << std::endl;
   //  std::cout << "after_2_uvox_new_2_0: " << after_2_uvox_new_2_0.size()  << std::endl;

 //   std::cout << "eeeee: " << eeee.size()  << std::endl;
 //   std::cout << "eeeee2: " << eeee2.size()  << std::endl;
   // std::cout << "-- TEST -- " << std::endl;


    /*
    VoxelGrid::array_type diff_grid;
    diff_grid.resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
    for (int zz = 0; zz < voxel_grid_->dimensions_.z(); ++zz) {
      for (int yy = 0; yy < voxel_grid_->dimensions_.y(); ++yy){
        for(int xx = 0; xx < voxel_grid_->dimensions_.x(); ++xx){
          if(start_grid_2[zz][yy][xx] != after_vg[zz][yy][xx]){
           // std::cout << "start_grid_2: " << start_grid_2[zz][yy][xx] << " != start_grid_: " << start_grid_[zz][yy][xx] << std::endl;

           // if(start_grid_2[zz][yy][xx] == 0)
              diff_grid[zz][yy][xx] = 2;
          }
        }
      }
    }
*/
  //  voxel_grid_->setVirtuelGrid(diff_grid);


    /*
    for (int zz = 0; zz < voxel_grid_->dimensions_.z(); ++zz) {
      for (int yy = 0; yy < voxel_grid_->dimensions_.y(); ++yy){
        for(int xx = 0; xx < voxel_grid_->dimensions_.x(); ++xx){
          start_grid_[zz][yy][xx] = -1;
        }
      }
    }
    */
  //  std::vector<Eigen::Vector3i> a;
  //  Eigen::Vector3i test(50,50,50);
  //  a.push_back(test);
  //  Eigen::Vector3i test2(60,60,60);
  //  a.push_back(test2);
  //  Eigen::Vector3i test3(70,70,40);
  //  a.push_back(test3);
  //  Eigen::Vector3i test4(30,30,30);
  //  a.push_back(test4);
  //  voxel_grid_->rayTraversal2(P0, a, start_grid_);






   // voxel_grid_->setVoxelGrid(start_grid_);


    // TEST //


/*
    std::vector<Tree::node*> childs = search_tree.getNodesOfDepth(2);
    ROS_INFO("number of childs: %d", (int)childs.size());

    voxel_grid_->setVoxelGrid(*root->child[0]->child[3]->voxel_grid);


    ROS_INFO("planner: %f %f %f", childs[0]->scan_position.x(),
             childs[0]->scan_position.y(),
             childs[0]->scan_position.z());


    std::vector<Tree::node*> childs2 = search_tree.getNodesOfDepth(1);
    ROS_INFO("Travel Cost: %d", childs2[7]->travel_cost);
    int p0_index = computeClosestPosistion(P0);
    int p1_index = computeClosestPosistion(childs2[7]->scan_position);
    ROS_INFO("Index p0: %d", p0_index);
    ROS_INFO("index p1: %d", p1_index);
*/

   // std::vector<Tree::node*> childs = search_tree.getNodesOfDepth(2);



 //   std::vector<Tree::node*> tt = search_tree.getNodesOfDepth(1);
 //   for(int i=0; i<(int)tt.size(); i++)
 //   {
 //     search_tree.print_short(tt[i]);
 //   }


   // Tree::node* t = search_tree.getNodesWithSmallestValue(childs);
   // std::cout << "---------------------" << std::endl;
   // search_tree.print_short(t);

    Tree::node* n = search_tree.getNextBestNode();
    search_tree.print_short(n);

    voxel_grid_->setVoxelGrid(*n->voxel_grid);
    //new_scan_position_ = new_scan_position;
    new_scan_position_ = n->scan_position;







//    new_scan_position_ = sampling_positions[index];



    double finish = clock();
    std::cerr << "Planning DONE in: "<< ( (finish - start)/CLOCKS_PER_SEC ) << std::endl;

    search_tree.cleanUp();

  }




  voxel_grid_->voxelGridMsg();
  return true;
}


bool Planner::plan_expectation(const Eigen::Vector3f& P0,
                   std::vector<Eigen::Vector3f>& sampling_positions)
{
  ROS_INFO("sampling positions: %d", (int)sampling_positions.size());

  // Markov Random Field (MRV) stuff
  voxel_grid_->createMRFs();
  voxel_grid_->evalMrf();
  voxel_grid_->MRFToVG();
  voxel_grid_->mrfVisul();

  ROS_INFO("Planner - plan : MRF - DONE");

  // Ray traversal
  VoxelGrid::array_type grid = voxel_grid_->getVoxelGrid();
  std::vector<Eigen::Vector3i> a = voxel_grid_->getUnknownVoxels(grid);
  voxel_grid_->rayTraversal2(P0, a, grid, true);
  voxel_grid_->setVoxelGrid(grid);
  ROS_INFO("Planner - plan: Ray traversal - DONE");

  std::vector<Eigen::Vector3i> new_scan_unknown_voxels = voxel_grid_->getUnknownVoxels(grid);
  ROS_INFO("NEW SCAN VOXEL SIZE: %d", (int)new_scan_unknown_voxels.size());

  double start = clock();

  //create the decision tree
  Tree search_tree;
  search_tree.setUVandCostWeight(1.0, 0.0);
  Tree::node* root = search_tree.getRoot();
  VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
  vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
  *vgrid = voxel_grid_->getVoxelGrid();
  root->voxel_grid = vgrid;
  root->sampling_positions = sampling_positions;

  for(int depth=0; depth<depth_; depth++)
  {
    std::vector<Tree::node*> childs = search_tree.getNodesOfDepth(depth);
    int n_childs = childs.size();

    for(int i_child=0; i_child<n_childs; i_child++)
    {
      Tree::node* parent = childs[i_child];

      ROS_INFO("size: %d", (int)parent->sampling_positions.size());
      for(unsigned int ii=0; ii<parent->sampling_positions.size(); ii++)
      {
        //get the voxel grid
        VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
        vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
        *vgrid = *parent->voxel_grid;

        std::vector<Eigen::Vector3i> uvox = voxel_grid_->getUnknownVoxels(*vgrid);
        ROS_INFO("uvox size: %d", (int)uvox.size());
        const Eigen::Vector3f P = parent->sampling_positions[ii];
        ROS_INFO("sample position index: %d", ii);
        voxel_grid_->estimateVoxelGrid(*vgrid, P, uvox);
        std::vector<Eigen::Vector3i> new_uvox = voxel_grid_->getUnknownVoxels(*vgrid);
        ROS_INFO("new_uvox size: %d", (int)new_uvox.size());

        //store the data in the tree
        int cost = calculateTravelCost(P0, parent->sampling_positions[ii]);
        double expectation = voxel_grid_->computeExpectation(*vgrid);
        ROS_INFO("expectation: %f", expectation);

        search_tree.newChild(parent, expectation, cost, vgrid, ii);
      }
    }




   int max_depth = root->max_depth;
    std::vector<Tree::node*> s_nodes = search_tree.getNodesOfDepth(max_depth);
    Tree::node* n = search_tree.getNodesWithSmallestValue(s_nodes);
    search_tree.print_short(n);

    //Tree::node* n = s_nodes[5];

    voxel_grid_->setVoxelGrid(*n->voxel_grid);
    new_scan_position_ = n->scan_position;
    sampling_positions = n->sampling_positions;

    double finish = clock();
    std::cerr << "Planning DONE in: "<< ( (finish - start)/CLOCKS_PER_SEC ) << std::endl;

    float expectation = (-1) * n->value;

    std::ifstream ifile("/home/potthast/projects/data/result.txt");
    std::ofstream file;
    if((bool)ifile)
    {
      //open file
      file.open("/home/potthast/projects/data/result.txt", std::ios::app);
    }else{
      //create a new file
      file.open("/home/potthast/projects/data/result.txt");
    }
    file << (int)new_scan_unknown_voxels.size() << " " << std::fixed << std::setprecision(3) << (float)expectation << "\n";
    file.close();

    std::cout << "Writing to disk - DONE" << std::endl;

    search_tree.cleanUp();

  }


  int index = computeClosestPosistion(full_sampling_positions_, new_scan_position_);
  ROS_INFO("scan number: %d", index+1);

  voxel_grid_->voxelGridMsg();
  return true;
}


bool Planner::plan_simple(const Eigen::Vector3f& P0,
                   std::vector<Eigen::Vector3f>& sampling_positions)
{
  ROS_INFO("sampling positions: %d", (int)sampling_positions.size());


  // Markov Random Field (MRV) stuff
  voxel_grid_->createMRFs();
  voxel_grid_->evalMrf();
  //voxel_grid_->MRFToVG();
  voxel_grid_->mrfVisul();
  ROS_INFO("Planner - plan : MRF - DONE");

  // Ray traversal
  VoxelGrid::array_type grid = voxel_grid_->getVoxelGrid();
  std::vector<Eigen::Vector3i> a = voxel_grid_->getUnknownVoxels(grid);
  voxel_grid_->rayTraversal2(P0, a, grid, true);
  voxel_grid_->setVoxelGrid(grid);
  ROS_INFO("Planner - plan: Ray traversal - DONE");

  std::vector<Eigen::Vector3i> new_scan_unknown_voxels = voxel_grid_->getUnknownVoxels(grid);
  ROS_INFO("NEW SCAN VOXEL SIZE: %d", (int)new_scan_unknown_voxels.size());

  double start = clock();

  //create the decision tree
  Tree search_tree;
  search_tree.setUVandCostWeight(1.0, 0.0);
  Tree::node* root = search_tree.getRoot();
  VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
  vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
  *vgrid = voxel_grid_->getVoxelGrid();
  root->voxel_grid = vgrid;
  root->sampling_positions = sampling_positions;

  for(int depth=0; depth<depth_; depth++)
  {
    std::vector<Tree::node*> childs = search_tree.getNodesOfDepth(depth);
    int n_childs = childs.size();

    for(int i_child=0; i_child<n_childs; i_child++)
    {
      Tree::node* parent = childs[i_child];

      ROS_INFO("size: %d", (int)parent->sampling_positions.size());
      for(unsigned int ii=0; ii<parent->sampling_positions.size(); ii++)
      {
        //get the voxel grid
        VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
        vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
        *vgrid = *parent->voxel_grid;

        std::vector<Eigen::Vector3i> uvox = voxel_grid_->getUnknownVoxels(*vgrid);
        ROS_INFO("uvox size: %d", (int)uvox.size());
        const Eigen::Vector3f P = parent->sampling_positions[ii];
        ROS_INFO("sample position index: %d", ii);
        voxel_grid_->rayTraversal2(P, uvox, *vgrid, true);
        std::vector<Eigen::Vector3i> new_uvox = voxel_grid_->getUnknownVoxels(*vgrid);
        ROS_INFO("new_uvox size: %d", (int)new_uvox.size());

        //store the data in the tree
        int cost = calculateTravelCost(P0, parent->sampling_positions[ii]);
        //double expectation = voxel_grid_->computeExpectation(*vgrid);
        ROS_INFO("CHANGE: %d", (int)new_scan_unknown_voxels.size()-(int)new_uvox.size());

        search_tree.newChild(parent, (int)new_uvox.size(), cost, vgrid, ii);
      }
    }




   int max_depth = root->max_depth;
    std::vector<Tree::node*> s_nodes = search_tree.getNodesOfDepth(max_depth);
    Tree::node* n = search_tree.getNodesWithSmallestValue(s_nodes);
    search_tree.print_short(n);

    //Tree::node* n = s_nodes[5];

    //voxel_grid_->setVoxelGrid(*n->voxel_grid);
    new_scan_position_ = n->scan_position;
    sampling_positions = n->sampling_positions;

    double finish = clock();
    std::cerr << "Planning DONE in: "<< ( (finish - start)/CLOCKS_PER_SEC ) << std::endl;

    float expectation = (-1) * n->value;
    if(expectation < 0.0)
      expectation = (-1) * expectation;

    std::ifstream ifile("/home/potthast/projects/data/result.txt");
    std::ofstream file;
    if((bool)ifile)
    {
      //open file
      file.open("/home/potthast/projects/data/result.txt", std::ios::app);
    }else{
      //create a new file
      file.open("/home/potthast/projects/data/result.txt");
    }
    file << (int)new_scan_unknown_voxels.size() << " " << std::fixed << std::setprecision(3) << (float)expectation << "\n";
    file.close();

    std::cout << "Writing to disk - DONE" << std::endl;

    search_tree.cleanUp();

  }

  int index = computeClosestPosistion(full_sampling_positions_, new_scan_position_);
  ROS_INFO("scan number: %d", index+1);

  voxel_grid_->voxelGridMsg();
  return true;
}





bool Planner::plan_greedy(const Eigen::Vector3f& P0,
                          std::vector<Eigen::Vector3f>& sampling_positions)
{

    double start = clock();

    int percent = 1;
    int total_vox = voxel_grid_->dimensions_.z()*voxel_grid_->dimensions_.y()*voxel_grid_->dimensions_.x();
    int stop_vox = total_vox*percent/100;

    stop_vox = 2500;

    // Markov Random Field (MRV) stuff
    voxel_grid_->createMRFs();
    voxel_grid_->evalMrf();
    voxel_grid_->MRFToVG();
    voxel_grid_->mrfVisul();
    ROS_INFO("Planner - plan : MRF - DONE");

    // Ray traversal
    VoxelGrid::array_type grid = voxel_grid_->getVoxelGrid();
    VoxelGrid::array_type start_grid_ = voxel_grid_->getVoxelGrid();
    std::vector<Eigen::Vector3i> a = voxel_grid_->getUnknownVoxels(grid);
    voxel_grid_->rayTraversal2(P0, a, grid, true);
    voxel_grid_->setVoxelGrid(grid);
    ROS_INFO("Planner - plan: Ray traversal - DONE");

    //create the decision tree
    Tree search_tree;
    search_tree.setUVandCostWeight(1.0, 0.0);
    Tree::node* root = search_tree.getRoot();
    VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
    vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
    *vgrid = voxel_grid_->getVoxelGrid();
    root->voxel_grid = vgrid;
    root->sampling_positions = sampling_positions;

    std::vector<Eigen::Vector3i> new_uvox;
    int depth = 0;
    Tree::node* parent = root;
    new_uvox = voxel_grid_->getUnknownVoxels(*parent->voxel_grid);

    std::cout << "new_uvox: " << (int)new_uvox.size()  << std::endl;
    std::cout << "stop_vox: " << stop_vox  << std::endl;

    while((int)new_uvox.size() > stop_vox)
    {
      std::cout << "new_uvox: " << (int)new_uvox.size()  << std::endl;

      ROS_INFO("size: %d", (int)parent->sampling_positions.size());
      for(unsigned int ii=0; ii<parent->sampling_positions.size(); ii++)
      {
        //get the voxel grid
        VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
        vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
        *vgrid = *parent->voxel_grid;

        std::vector<Eigen::Vector3i> uvox = voxel_grid_->getUnknownVoxels(*vgrid);
        //ROS_INFO("uvox size: %d", (int)uvox.size());
        const Eigen::Vector3f P = parent->sampling_positions[ii];
        voxel_grid_->estimateVoxelGrid(*vgrid, P, uvox);
        new_uvox = voxel_grid_->getUnknownVoxels(*vgrid);
        ROS_INFO("new_uvox size: %d", (int)new_uvox.size());

        //store the data in the tree
        int cost = calculateTravelCost(P0, parent->sampling_positions[ii]);
        search_tree.newChild(parent, (int)new_uvox.size(), cost, vgrid, ii);
      }
      depth++;

      std::vector<Tree::node*> childs = search_tree.getNodesOfDepth(depth);
      Tree::node* smallest_node = search_tree.getNodesWithSmallestValue(childs);
      new_uvox = voxel_grid_->getUnknownVoxels(*smallest_node->voxel_grid);
      parent = smallest_node;

      if(smallest_node->sampling_positions.size() == 0){
        break;
      }

    }




  /*
    std::vector<Tree::node*> bla = search_tree.getNodesOfDepth(4);
       for(int i=0; i<(int)bla.size(); i++)
       {
         search_tree.print_short(bla[i]);
       }
  */

    std::cout << "max_depth: " << root->max_depth  << std::endl;



    shortest_path_ = search_tree.getshortestPath(P0);

    std::cout << "shortest_path_: " << (int)shortest_path_.size()  << std::endl;

    double finish = clock();
    std::cerr << "Planning DONE in: "<< ( (finish - start)/CLOCKS_PER_SEC ) << std::endl;

    //search_tree.cleanUp();


    std::vector<Tree::node*> path;
      path = search_tree.getPath();

      /*
    for(int i=0; i<(int)path.size(); i++)
    {
      std::cout << path[i]->scan_position << std::endl;
    }
*/





  voxel_grid_->voxelGridMsg();

  if((int)shortest_path_.size() > 0)
  {
    new_scan_position_ = shortest_path_[0]->scan_position;
    int index = computeClosestPosistion(sampling_positions, new_scan_position_);
    sampling_positions.erase(sampling_positions.begin() + index);
  //  voxel_grid_->setVoxelGrid(*shortest_path_[0]->voxel_grid);



    shortest_path_.erase(shortest_path_.begin());

  }else{
    return false;
  }

  return true;
}



/////////////////////////////////////////////////////////////////////////////////
// Field of view limitation planning //
/////////////////////////////////////////////////////////////////////////////////
bool Planner::plan_simple_FOV(const Eigen::Vector3f& P0,
                   std::vector<Eigen::Vector3f>& sampling_positions,
                   std::vector<std::vector<float> >& sampling_angle,
                   float direction)
{
  // delete the direction from the vector
  int indx = computeClosestPosistion(full_sampling_positions_, P0);
  for(unsigned int i=0; i < sampling_angle[indx].size(); i++)
  {
    if(sampling_angle[indx][i] == direction)
    {
      sampling_angle[indx].erase(sampling_angle[indx].begin() + i);
      break;
    }
  }

  ROS_INFO("sampling positions: %d", (int)sampling_positions.size());


  // Markov Random Field (MRV) stuff
  voxel_grid_->createMRFs();
  voxel_grid_->evalMrf();
  voxel_grid_->MRFToVG();
  voxel_grid_->mrfVisul();
  ROS_INFO("Planner - plan : MRF - DONE");

  // Ray traversal
  VoxelGrid::array_type grid = voxel_grid_->getVoxelGrid();
  //std::vector<Eigen::Vector3i> a = voxel_grid_->getUnknownVoxels(grid);
  std::vector<Eigen::Vector3i> a = voxel_grid_->getUnknownVoxelsFOV(P0,grid, direction);
  voxel_grid_->rayTraversal2(P0, a, grid, true);
  voxel_grid_->setVoxelGrid(grid);
  ROS_INFO("Planner - plan: Ray traversal - DONE");

  std::vector<Eigen::Vector3i> new_scan_unknown_voxels = voxel_grid_->getUnknownVoxels(grid);
  ROS_INFO("NEW SCAN VOXEL SIZE: %d", (int)new_scan_unknown_voxels.size());

  double start = clock();

  //create the decision tree
  Tree search_tree;
  search_tree.setUVandCostWeight(1.0, 0.0);
  Tree::node* root = search_tree.getRoot();
  VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
  vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
  *vgrid = voxel_grid_->getVoxelGrid();
  root->voxel_grid = vgrid;
  root->sampling_positions = sampling_positions;

  for(int depth=0; depth<depth_; depth++)
  {
    std::vector<Tree::node*> childs = search_tree.getNodesOfDepth(depth);
    int n_childs = childs.size();

    for(int i_child=0; i_child<n_childs; i_child++)
    {
      Tree::node* parent = childs[i_child];

      ROS_INFO("size: %d", (int)parent->sampling_positions.size());
      for(unsigned int ii=0; ii<parent->sampling_positions.size(); ii++)
      {
        for(unsigned int j=0; j<sampling_angle[ii].size(); j++)
        {
          //get the voxel grid
          VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
          vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
          *vgrid = *parent->voxel_grid;

          //std::vector<Eigen::Vector3i> uvox = voxel_grid_->getUnknownVoxelsFOV(P0,*vgrid, direction);
          //ROS_INFO("uvox size: %d", (int)uvox.size());
          const Eigen::Vector3f P = parent->sampling_positions[ii];
          std::vector<Eigen::Vector3i> uvox = voxel_grid_->getUnknownVoxelsFOV(P,*vgrid, sampling_angle[ii][j]);
          ROS_INFO("sample position index: %d", ii);
          ROS_INFO("sample direction: %f", sampling_angle[ii][j]);
          voxel_grid_->rayTraversal2(P, uvox, *vgrid, true);
          //std::vector<Eigen::Vector3i> new_uvox = voxel_grid_->getUnknownVoxelsFOV(P0,*vgrid,j);
          //ROS_INFO("new_uvox size: %d", (int)new_uvox.size());


          std::vector<Eigen::Vector3i> new_voxels = voxel_grid_->getUnknownVoxels(*vgrid);
          //store the data in the tree
          int cost = calculateTravelCost(P0, parent->sampling_positions[ii]);
          //double expectation = voxel_grid_->computeExpectation(*vgrid);
          ROS_INFO("CHANGE: %d", (int)new_scan_unknown_voxels.size()-(int)new_voxels.size());

          search_tree.newChild(parent, (int)new_voxels.size(), cost, vgrid, ii, sampling_angle[ii][j]);
        }
      }
    }




   int max_depth = root->max_depth;
    std::vector<Tree::node*> s_nodes = search_tree.getNodesOfDepth(max_depth);
    Tree::node* n = search_tree.getNodesWithSmallestValue(s_nodes);
    search_tree.print_short(n);

    //Tree::node* n = s_nodes[5];

    //voxel_grid_->setVoxelGrid(*n->voxel_grid);
    new_scan_position_ = n->scan_position;
    //sampling_positions = n->sampling_positions;
    direction_ = n->direction;

    double finish = clock();
    std::cerr << "Planning DONE in: "<< ( (finish - start)/CLOCKS_PER_SEC ) << std::endl;

    float expectation = (-1) * n->value;
    if(expectation < 0.0)
      expectation = (-1) * expectation;

    std::ifstream ifile("/home/potthast/projects/data/result.txt");
    std::ofstream file;
    if((bool)ifile)
    {
      //open file
      file.open("/home/potthast/projects/data/result.txt", std::ios::app);
    }else{
      //create a new file
      file.open("/home/potthast/projects/data/result.txt");
    }
    file << (int)new_scan_unknown_voxels.size() << " " << std::fixed << std::setprecision(3) << (float)expectation << "\n";
    file.close();

    std::cout << "Writing to disk - DONE" << std::endl;

    search_tree.cleanUp();

  }

  grid = voxel_grid_->getVoxelGrid();
  a = voxel_grid_->getUnknownVoxels(grid);

  float percent = voxel_grid_->dimensions_.x() * voxel_grid_->dimensions_.y() * voxel_grid_->dimensions_.z();
  percent = (float)a.size() / percent;
  percent = percent * 100.0;

  ROS_INFO("Unknown voxels: %d ; percent: %f", (int)a.size(), percent);


  int index = computeClosestPosistion(full_sampling_positions_, new_scan_position_);
  ROS_INFO("scan number: %d", index+1);
  ROS_INFO("direction: %f", direction_);

  voxel_grid_->voxelGridMsg();
  return true;
}

bool Planner::plan_expectation_FOV(const Eigen::Vector3f& P0,
                                   std::vector<Eigen::Vector3f>& sampling_positions,
                                   std::vector<std::vector<float> > &sampling_angle,
                                   float direction)
{
  new_scan_position_.x() = 0.0;
  new_scan_position_.y() = 0.0;
  new_scan_position_.z() = 0.0;

  // delete the direction from the vector
  int indx = computeClosestPosistion(P0);
  for(unsigned int i=0; i < sampling_angle[indx].size(); i++)
  {
    if(sampling_angle[indx][i] == direction)
    {
      sampling_angle[indx].erase(sampling_angle[indx].begin() + i);
      break;
    }
  }


  ROS_INFO("sampling positions: %d", (int)sampling_positions.size());

  // Markov Random Field (MRV) stuff
  voxel_grid_->createMRFs();
  voxel_grid_->evalMrf();
  voxel_grid_->MRFToVG();
  voxel_grid_->mrfVisul();

  ROS_INFO("Planner - plan : MRF - DONE");

  // Ray traversal
  VoxelGrid::array_type grid = voxel_grid_->getVoxelGrid();
  //std::vector<Eigen::Vector3i> a = voxel_grid_->getUnknownVoxels(grid);
  std::vector<Eigen::Vector3i> a = voxel_grid_->getUnknownVoxelsFOV(P0,grid, direction);
  voxel_grid_->rayTraversal2(P0, a, grid, true);
  voxel_grid_->setVoxelGrid(grid);
  ROS_INFO("Planner - plan: Ray traversal - DONE");

  std::vector<Eigen::Vector3i> new_scan_unknown_voxels = voxel_grid_->getUnknownVoxels(grid);
  ROS_INFO("NEW SCAN VOXEL SIZE: %d", (int)new_scan_unknown_voxels.size());


 // voxel_grid_->computeUnknownObjectProb(new_scan_unknown_voxels);
  voxel_grid_->setObservationProb(new_scan_unknown_voxels, 0.0);


  double start = clock();
  //create the decision tree
  Tree search_tree;
  search_tree.setUVandCostWeight(1.0, 0.0);
  Tree::node* root = search_tree.getRoot();
  VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
  vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
  *vgrid = voxel_grid_->getVoxelGrid();
  root->voxel_grid = vgrid;
  root->sampling_positions = sampling_positions;

  for(int depth=0; depth<depth_; depth++)
  {
    std::vector<Tree::node*> childs = search_tree.getNodesOfDepth(depth);
    int n_childs = childs.size();

    for(int i_child=0; i_child<n_childs; i_child++)
    {
      Tree::node* parent = childs[i_child];

      ROS_INFO("size: %d", (int)parent->sampling_positions.size());
      for(unsigned int ii=0; ii<parent->sampling_positions.size(); ii++)
      {
        for(unsigned int j=0; j<sampling_angle[ii].size(); j++)
        {
          //get the voxel grid
          VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
          vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
          *vgrid = *parent->voxel_grid;

          //std::vector<Eigen::Vector3i> uvox = voxel_grid_->getUnknownVoxels(*vgrid);
          //std::vector<Eigen::Vector3i> uvox = voxel_grid_->getUnknownVoxelsFOV(P0,*vgrid, direction);
          //ROS_INFO("uvox size: %d", (int)uvox.size());
          const Eigen::Vector3f P = parent->sampling_positions[ii];
          std::vector<Eigen::Vector3i> uvox = voxel_grid_->getUnknownVoxelsFOV(P,*vgrid, sampling_angle[ii][j]);
          ROS_INFO("sample position index: %d", ii);
          ROS_INFO("sample direction: %f", sampling_angle[ii][j]);
          voxel_grid_->estimateVoxelGrid(*vgrid, P, uvox);
          //std::vector<Eigen::Vector3i> new_uvox = voxel_grid_->getUnknownVoxels(*vgrid);
          //std::vector<Eigen::Vector3i> new_uvox = voxel_grid_->getUnknownVoxelsFOV(P0,*vgrid,j);
          //ROS_INFO("new_uvox size: %d", (int)new_uvox.size());

          //store the data in the tree
          int cost = calculateTravelCost(P0, parent->sampling_positions[ii]);
          double expectation = voxel_grid_->computeExpectation(*vgrid, uvox);
          ROS_INFO("expectation: %f", expectation);

          search_tree.newChild(parent, expectation, cost, vgrid, ii, sampling_angle[ii][j]);
        }
      }
    }




   int max_depth = root->max_depth;
    std::vector<Tree::node*> s_nodes = search_tree.getNodesOfDepth(max_depth);
    Tree::node* n = search_tree.getNodesWithSmallestValue(s_nodes);
    search_tree.print_short(n);

    //Tree::node* n = s_nodes[5];

    voxel_grid_->setVoxelGrid(*n->voxel_grid);
    new_scan_position_ = n->scan_position;
    //sampling_positions = n->sampling_positions;
    direction_ = n->direction;

    double finish = clock();
    std::cerr << "Planning DONE in: "<< ( (finish - start)/CLOCKS_PER_SEC ) << std::endl;

    float expectation = (-1) * n->value;

    std::ifstream ifile("/home/potthast/projects/data/result.txt");
    std::ofstream file;
    if((bool)ifile)
    {
      //open file
      file.open("/home/potthast/projects/data/result.txt", std::ios::app);
    }else{
      //create a new file
      file.open("/home/potthast/projects/data/result.txt");
    }
    file << (int)new_scan_unknown_voxels.size() << " " << std::fixed << std::setprecision(3) << (float)expectation << "\n";
    file.close();

    std::cout << "Writing to disk - DONE" << std::endl;

    search_tree.cleanUp();

  }


  int index = computeClosestPosistion(full_sampling_positions_, new_scan_position_);
  ROS_INFO("scan number: %d", index+1);
  ROS_INFO("direction: %f", direction_);

  voxel_grid_->voxelGridMsg();
  return true;
}

bool Planner::random_FOV(const Eigen::Vector3f& P0,
                   std::vector<Eigen::Vector3f>& sampling_positions,
                   std::vector<std::vector<float> >& sampling_angle,
                   float direction)
{

  // delete the direction from the vector
  int indx = computeClosestPosistion(full_sampling_positions_, P0);
  for(unsigned int i=0; i < sampling_angle[indx].size(); i++)
  {
    if(sampling_angle[indx][i] == direction)
    {
      sampling_angle[indx].erase(sampling_angle[indx].begin() + i);
      break;
    }
  }

  ROS_INFO("sampling positions: %d", (int)sampling_positions.size());


  // Markov Random Field (MRV) stuff
  voxel_grid_->createMRFs();
  voxel_grid_->evalMrf();
  voxel_grid_->MRFToVG();
  voxel_grid_->mrfVisul();
  ROS_INFO("Planner - plan : MRF - DONE");

  // Ray traversal
  VoxelGrid::array_type grid = voxel_grid_->getVoxelGrid();
  //std::vector<Eigen::Vector3i> a = voxel_grid_->getUnknownVoxels(grid);
  std::vector<Eigen::Vector3i> a = voxel_grid_->getUnknownVoxelsFOV(P0,grid, direction);
  voxel_grid_->rayTraversal2(P0, a, grid, true);
  voxel_grid_->setVoxelGrid(grid);
  ROS_INFO("Planner - plan: Ray traversal - DONE");

  std::vector<Eigen::Vector3i> new_scan_unknown_voxels = voxel_grid_->getUnknownVoxels(grid);
  ROS_INFO("NEW SCAN VOXEL SIZE: %d", (int)new_scan_unknown_voxels.size());

  double start = clock();

  /*
  //create the decision tree
  Tree search_tree;
  search_tree.setUVandCostWeight(1.0, 0.0);
  Tree::node* root = search_tree.getRoot();
  VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
  vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
  *vgrid = voxel_grid_->getVoxelGrid();
  root->voxel_grid = vgrid;
  root->sampling_positions = sampling_positions;

  for(int depth=0; depth<depth_; depth++)
  {
    std::vector<Tree::node*> childs = search_tree.getNodesOfDepth(depth);
    int n_childs = childs.size();

    for(int i_child=0; i_child<n_childs; i_child++)
    {
      Tree::node* parent = childs[i_child];

      ROS_INFO("size: %d", (int)parent->sampling_positions.size());
      for(unsigned int ii=0; ii<parent->sampling_positions.size(); ii++)
      {
        for(unsigned int j=0; j<sampling_angle[ii].size(); j++)
        {
          //get the voxel grid
          VoxelGrid::array_type* vgrid = new VoxelGrid::array_type;
          vgrid->resize(boost::extents[voxel_grid_->dimensions_.z()][voxel_grid_->dimensions_.y()][voxel_grid_->dimensions_.x()]);
          *vgrid = *parent->voxel_grid;

          //std::vector<Eigen::Vector3i> uvox = voxel_grid_->getUnknownVoxelsFOV(P0,*vgrid, direction);
          //ROS_INFO("uvox size: %d", (int)uvox.size());
          const Eigen::Vector3f P = parent->sampling_positions[ii];
          std::vector<Eigen::Vector3i> uvox = voxel_grid_->getUnknownVoxelsFOV(P,*vgrid, sampling_angle[ii][j]);
          ROS_INFO("sample position index: %d", ii);
          ROS_INFO("sample direction: %f", sampling_angle[ii][j]);
          voxel_grid_->rayTraversal2(P, uvox, *vgrid, true);
          //std::vector<Eigen::Vector3i> new_uvox = voxel_grid_->getUnknownVoxelsFOV(P0,*vgrid,j);
          //ROS_INFO("new_uvox size: %d", (int)new_uvox.size());


          std::vector<Eigen::Vector3i> new_voxels = voxel_grid_->getUnknownVoxels(*vgrid);
          //store the data in the tree
          int cost = calculateTravelCost(P0, parent->sampling_positions[ii]);
          //double expectation = voxel_grid_->computeExpectation(*vgrid);
          ROS_INFO("CHANGE: %d", (int)new_scan_unknown_voxels.size()-(int)new_voxels.size());

          search_tree.newChild(parent, (int)new_voxels.size(), cost, vgrid, ii, sampling_angle[ii][j]);
        }
      }
    }




   int max_depth = root->max_depth;
    std::vector<Tree::node*> s_nodes = search_tree.getNodesOfDepth(max_depth);
    Tree::node* n = search_tree.getNodesWithSmallestValue(s_nodes);
    search_tree.print_short(n);

    //Tree::node* n = s_nodes[5];

    //voxel_grid_->setVoxelGrid(*n->voxel_grid);
    new_scan_position_ = n->scan_position;
    //sampling_positions = n->sampling_positions;
    direction_ = n->direction;
    */

  // randomly choose a sampling direction
  int pos_index;
  bool c = false;
  boost::uniform_int<> gm(1, 3);
  boost::uniform_int<> gmm(1, 10);
  while(!c)
  {
    float dir [3] = { -15.0, 0.0, 15.0 };

    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > vg( gen_, gm );
    int dir_index = vg() - 1;
    direction = dir[dir_index];
    direction_ = direction;
    // randomly choose a sampling position

    ROS_INFO("bbbbb: %i", dir_index);
    ROS_INFO("dddddd: %f", direction);


    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > vg2( gen_, gmm );
    pos_index = vg2() - 1;

    ROS_INFO("aaaaa: %i", pos_index);

    for(unsigned int i=0; i < sampling_angle[pos_index].size(); i++)
    {
      if(sampling_angle[pos_index][i] == direction)
        c = true;
    }
  }

  new_scan_position_ = full_sampling_positions_[pos_index];

    double finish = clock();
    std::cerr << "Planning DONE in: "<< ( (finish - start)/CLOCKS_PER_SEC ) << std::endl;
/*
    float expectation = (-1) * n->value;
    if(expectation < 0.0)
      expectation = (-1) * expectation;
*/
    std::ifstream ifile("/home/potthast/projects/data/result.txt");
    std::ofstream file;
    if((bool)ifile)
    {
      //open file
      file.open("/home/potthast/projects/data/result.txt", std::ios::app);
    }else{
      //create a new file
      file.open("/home/potthast/projects/data/result.txt");
    }
    file << (int)new_scan_unknown_voxels.size() << "\n";
    file.close();

    std::cout << "Writing to disk - DONE" << std::endl;

  //  search_tree.cleanUp();

 // }

  //int index = computeClosestPosistion(full_sampling_positions_, new_scan_position_);

  //ROS_INFO("scan number: %d", index+1);
  ROS_INFO("direction: %f", direction_);

  voxel_grid_->voxelGridMsg();
  return true;
}




/////////////////////////////////////////////////////////////////////////////////
// Other stuff //
/////////////////////////////////////////////////////////////////////////////////

bool Planner::collect_scans(const Eigen::Vector3f& P0,
                          std::vector<Eigen::Vector3f>& sampling_positions)
{

  Eigen::Vector3f pos = sampling_positions[0];
  new_scan_position_ = pos;

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
