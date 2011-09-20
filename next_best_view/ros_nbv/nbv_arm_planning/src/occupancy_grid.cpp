/*
 * occupancy_grid.cpp
 *
 *  Created on: Apr 2, 2011
 *      Author: potthast
 */

#include <nbv_arm_planning/occupancy_grid.h>

OccupancyGrid::OccupancyGrid(Eigen::Vector3f box, Eigen::Vector3i dim,
                             Eigen::Vector4f centroid, std::vector<Eigen::Vector3f> table_top_bbx,
                             PublishTopics& publish_topics)
{
  publish_topics_ = &publish_topics;

  dimensions_ = Eigen::Vector3i(dim.x(), dim.y(), dim.z());
  bbx_ = Eigen::Vector3f(box.x(), box.y(), box.z());
  centroid_ = centroid;
  voxel_ = Eigen::Vector3f(box.x() / dim.x(), box.y() / dim.y(), box.z() / dim.z());
  centroid_voxel_ = Eigen::Vector3f( (dim.x()/2) , (dim.y()/2), (dim.z()/2) );

  ROS_INFO("box size: %f %f %f", box.x(), box.y(), box.z());
  ROS_INFO("voxel size: %f %f %f", voxel_.x(), voxel_.y(), voxel_.z());

  occupancy_grid_.resize(boost::extents[dimensions_.z()][dimensions_.y()][dimensions_.x()]);

  //tmp_grid_.resize(boost::extents[dimensions_.z()][dimensions_.y()][dimensions_.x()]);

  // initialize all fields with 0.5 (unknown);
  voxel_t v;
  for (int zz = 0; zz < dimensions_.z(); ++zz) {
    for (int yy = 0; yy < dimensions_.y(); ++yy){
      for(int xx = 0; xx < dimensions_.x(); ++xx){
        v.p = 0.5;
        v.update = false;
        v.state = UNKNOWN;
        v.observation_prob = 0.5;
        v.enclosed_prob = 1.0;
        occupancy_grid_[zz][yy][xx] = v;
      }
    }
  }

  publish_topics_->voxelGridBbxMsg(table_top_bbx, bbx_);
}

void OccupancyGrid::measurementsUpdate()
{
  Eigen::MatrixXf sigma(3,3);
  Eigen::MatrixXf sigmaInv(3,3);

  sigma << 0.2, 0, 0,
           0, 0.2, 0,
           0, 0, 0.2;
  sigmaInv = sigma.inverse();

  for( int i=0; i < (int)measurement_->points.size(); i++){
    Eigen::Vector3f m(measurement_->points[i].x, measurement_->points[i].y, measurement_->points[i].z);
    Eigen::Vector3i coord = cartesianToVoxel(m);

    //because the point cloud is not exact horizontal
    // this has the effect that the objects are slightly
    // non horizontal
    if(coord.z() < 0){
      //ROS_INFO("setGridPoints - coord x: %d y: %d z: %d", coord.x(), coord.y(), coord.z());
      coord.z() = 0;
    }

    if(coord.z() > dimensions_.z()-1)
    {
      coord.z() = dimensions_.z() - 1;
    }

    if(coord.x() < 0)
    {
      coord.x() = 0;
    }

    if(coord.x() > dimensions_.x()-1)
    {
      coord.x() = dimensions_.x() - 1;
    }


    if(coord.y() < 0)
    {
      coord.y() = 0;
    }

    if(coord.y() > dimensions_.y()-1)
    {
      coord.y() = dimensions_.y() - 1;
    }

    //ROS_INFO("%d %d %d", coord.x(), coord.y(), coord.z());

    Eigen::Vector3f mu = voxelToCartesian(coord);
    float p_z_given_r = multivariateGaussianDistribution(m, mu, sigma, sigmaInv);
    float p_r_given_z = occupancy_grid_[coord.z()][coord.y()][coord.x()].p;
    float p = bayesianUpdate(p_z_given_r, p_r_given_z);

    //voxel_t voxel;
    //voxel.p = p;
    //voxel.update = true;
    //voxel.state = occupancy_grid_[coord.z()][coord.y()][coord.x()].state;

    occupancy_grid_[coord.z()][coord.y()][coord.x()].p = p;
    occupancy_grid_[coord.z()][coord.y()][coord.x()].update = true;

  }


}


void OccupancyGrid::createMRFs()
{
  mrfs_.clear();

  for(int i=0; i<(int)objd_.size(); i++)
  {
    array_type mrf;

    int h, w, d;
    Eigen::Vector3f P0(objd_[i](0), objd_[i](1), objd_[i](2));
    Eigen::Vector3f P1(objd_[i](3), objd_[i](4), objd_[i](5));

    d = (cartesianToVoxel(P1).x() - cartesianToVoxel(P0).x()) +1;
    w = (cartesianToVoxel(P1).y() - cartesianToVoxel(P0).y()) +1;
    h = (cartesianToVoxel(P1).z() - cartesianToVoxel(P0).z()) +1;

    mrf.resize(boost::extents[h][w][d]);

    for (int zz = 0; zz < h; ++zz) {
      for (int yy = 0; yy < w; ++yy){
        for(int xx = 0; xx < d; ++xx){
          int z = cartesianToVoxel(P0).z() + zz;
          int y = cartesianToVoxel(P0).y() + yy;
          int x = cartesianToVoxel(P0).x() + xx;

          if(z < 0)
            z = 0;

          if(y < 0)
            y = 0;

          if(x < 0)
            x = 0;

          if(z > dimensions_.z()-1)
          {
            z = dimensions_.z() - 1;
          }

          //ROS_INFO("x: %d, y: %d, z: %d", x, y, z);

          voxel_t v;
          if( occupancy_grid_[z][y][x].state == FREE || occupancy_grid_[z][y][x].state == UNKNOWN){
            v.state = -1;
            v.observation_prob = 0.0;
            mrf[zz][yy][xx] = v;
          }else if( occupancy_grid_[z][y][x].state == OCCUPIED ){
            v.state = OCCUPIED;
            mrf[zz][yy][xx] = v;
          }
        }
      }
    }

    mrfs_.push_back(mrf);

  }
}

void OccupancyGrid::evalMrf()
{
  for( int i=0; i<(int) mrfs_.size(); i++)
  {
    array_type hl;

    int h, w, d;
    const Eigen::Vector3f P0(objd_[i](0), objd_[i](1), objd_[i](2));
    const Eigen::Vector3f P1(objd_[i](3), objd_[i](4), objd_[i](5));
    d = (cartesianToVoxel(P1).x() - cartesianToVoxel(P0).x()) +1;
    w = (cartesianToVoxel(P1).y() - cartesianToVoxel(P0).y()) +1;
    h = (cartesianToVoxel(P1).z() - cartesianToVoxel(P0).z()) +1;

    hl.resize(boost::extents[h][w][d]);
    hl = mrfs_[i];

    Eigen::Vector3i dim(d, w, h);

    ICM(mrfs_[i], hl, dim);

    mrfs_[i] = hl;
  }

}

void OccupancyGrid::ICM(array_type &mrf, array_type &hl, Eigen::Vector3i &dim)
{
  int iter = 0;
  bool converge = false;
  double beta = 1.0;



  int d = dim.x();
  int w = dim.y();
  int h = dim.z();

  /*
  double energy = 0.0;
  for (int zz = 0; zz < h; ++zz) {
    for (int yy = 0; yy < w; ++yy){
      for(int xx = 0; xx < d; ++xx){
        energy += hl[zz][yy][xx] * mrf[zz][yy][xx];

      }
    }
  }

  energy = 2.1 * energy;


  std::cerr << "    begin energy: "<< energy <<  std::endl;
   */

  //  while(converge==false){
  for(int jj=0; jj<5; jj++){
    double energy = 0.0;
    converge = true;

    std::vector<int> nb;
    std::vector<int> mrfnb;
    for (int zz = 0; zz < h; ++zz) {
      for (int yy = 0; yy < w; ++yy){
        for(int xx = 0; xx < d; ++xx){
          int v = hl[zz][yy][xx].state;

          if(mrf[zz][yy][xx].state != OCCUPIED){

            // extract the neighbors from the fixed voxel
            nb.clear();
            if( xx < d-1){
              nb.push_back(hl[zz][yy][xx+1].state);
            }else
              nb.push_back(0);
            if( xx > 0){
              nb.push_back(hl[zz][yy][xx-1].state);
            }else
              nb.push_back(0);
            if( yy < w-1){
              nb.push_back(hl[zz][yy+1][xx].state);
            }else
              nb.push_back(0);
            if( yy > 0){
              nb.push_back(hl[zz][yy-1][xx].state);
            }else
              nb.push_back(0);
            if( zz < h-1){
              nb.push_back(hl[zz+1][yy][xx].state);
            }else
              nb.push_back(0);
            if( zz > 0){
              nb.push_back(hl[zz-1][yy][xx].state);
            }else
              nb.push_back(0);



            // sum over the neighbors
            int sum_1 = 0;
            int sum_2 = 0;
            int energy_1 = 0;
            int energy_2 = 0;
            int v_2 = (-1)*v;
            for( int i=0; i<(int)nb.size(); i++){
              if( i==0 || i==2 || i==4 ){
                //  if( mrfnb[i] == OCCUPIED && mrfnb[i+1] == OCCUPIED)
                //    nb[i] = nb[i]*5;
                //}

                // indicator function
                int I = 1;
                if(nb[i] == OCCUPIED && nb[i+1] == OCCUPIED)
                  I = 2;

                // energy function
                sum_1 += I*(v*nb[i] + v*nb[i+1]);
                sum_2 += I*(v_2*nb[i] + v_2*nb[i+1]);


              }

              //   sum_1 += v*nb[i] ;
              //   sum_2 += v_2*nb[i];



            }

            energy_1 = -beta*sum_1;
            energy_2 = -beta*sum_2;

            /*
        sum_2 = 0;
        int energy_2 = 0;
        int v_2 = (-1)*v;
        for( int i=0; i<(int)nb.size(); i++){
          sum_2 += v_2*nb[i];
        }
        energy_2 = -beta*sum_2;
             */
            //  std::cerr << energy_1 << "   " << energy_2 << std::endl;



            // change value
            if(energy_2 <= energy_1){
              hl[zz][yy][xx].state = v_2;
              converge = false;

              energy += energy_2;

              //   std::cerr << "change: "<< v_2 << std::endl;
            }else{
              energy += energy_1;

            }


            energy += -2.1*(hl[zz][yy][xx].state * mrf[zz][yy][xx].state);



          }
        }
      }

    }

    iter++;
    std::cerr << iter << "    energy: "<< energy <<  std::endl;

  }

  std::cerr << "MRF: DONE" << "    Iterations: " << iter << std::endl;

}

void OccupancyGrid::MRFToVG()
{
  for(int i=0; i<(int)objd_.size(); i++)
  {
    int h, w, d;
    const Eigen::Vector3f P0(objd_[i](0), objd_[i](1), objd_[i](2));
    const Eigen::Vector3f P1(objd_[i](3), objd_[i](4), objd_[i](5));
    d = (cartesianToVoxel(P1).x() - cartesianToVoxel(P0).x()) +1;
    w = (cartesianToVoxel(P1).y() - cartesianToVoxel(P0).y()) +1;
    h = (cartesianToVoxel(P1).z() - cartesianToVoxel(P0).z()) +1;

    for (int zz = 0; zz < h; ++zz) {
      for (int yy = 0; yy < w; ++yy){
        for(int xx = 0; xx < d; ++xx){
          int z = cartesianToVoxel(P0).z() + zz;
          int y = cartesianToVoxel(P0).y() + yy;
          int x = cartesianToVoxel(P0).x() + xx;

          if(z < 0)
            z = 0;

          if(y < 0)
            y = 0;

          if(x < 0)
            x = 0;

          if(z > dimensions_.z()-1)
          {
            z = dimensions_.z() - 1;
          }

          //ROS_INFO("x: %d, y: %d, z: %d", x, y, z);

          if(mrfs_[i][zz][yy][xx].state == OCCUPIED){
            occupancy_grid_[z][y][x].state = mrfs_[i][zz][yy][xx].state;
            occupancy_grid_[z][y][x].observation_prob = mrfs_[i][zz][yy][xx].observation_prob;
          }else{
            if(occupancy_grid_[z][y][x].state == UNKNOWN)
              occupancy_grid_[z][y][x].enclosed_prob = ENCLOSED_PROB;
          }

        }
      }
    }
  }

}



void OccupancyGrid::setStates()
{
  for (int zz = dimensions_.z()-1; zz >(-1) ; zz--) {
    for (int yy = 0; yy < dimensions_.y(); ++yy){
      for(int xx = 0; xx < dimensions_.x(); ++xx){
        if( occupancy_grid_[zz][yy][xx].p > 0.4 && occupancy_grid_[zz][yy][xx].p < 0.6)
        {
          occupancy_grid_[zz][yy][xx].state = UNKNOWN;
        }
        if( occupancy_grid_[zz][yy][xx].p < 0.4)
        {
          occupancy_grid_[zz][yy][xx].state = FREE;
        }
        if( occupancy_grid_[zz][yy][xx].p > 0.6)
        {
          occupancy_grid_[zz][yy][xx].state = OCCUPIED;
          occupancy_grid_[zz][yy][xx].observation_prob = 0.0;
        }
      }
    }
  }
}

std::vector<Eigen::Vector3i> OccupancyGrid::getUnknownVoxels(array_type &grid)
{
  std::vector<Eigen::Vector3i> uvox;

  for (int zz = dimensions_.z()-1; zz >(-1) ; zz--) {
    for (int yy = 0; yy < dimensions_.y(); ++yy){
      for(int xx = 0; xx < dimensions_.x(); ++xx){
        if( (grid[zz][yy][xx].p > 0.4 && grid[zz][yy][xx].p < 0.6) || grid[zz][yy][xx].state == UNKNOWN)
        //if( grid[zz][yy][xx].state == UNKNOWN)
        {
          Eigen::Vector3i vox(xx, yy, zz);
          uvox.push_back(vox);
        }
      }
    }
  }
  return uvox;
}

std::vector<Eigen::Vector3i> OccupancyGrid::getUnknownVoxelsFOV(const Eigen::Vector3f P0, array_type &grid, float direction)
{
  // field of view angle
  //float fov = 32.0;
  float fov = 29.0;

  Eigen::Vector2f p1(view_dir_.x(), view_dir_.y());

 // Eigen::Vector2f p1(centroid_.x(), centroid_.y());
  Eigen::Vector2f p2(P0.x(), P0.y());

  /*
  float theta = direction * M_PI/180;
  Eigen::Matrix2f R;
  R << cos(theta), -sin(theta),
       sin(theta),  cos(theta);

  Eigen::Vector2f p2f(0.0,0.0);
  p2f = R*(p1-p2) + p2;
  p1 = p2f;
*/

  Eigen::Vector2f normal = computeNormal(p1,p2,fov);

  Eigen::Vector3f test(p1.x(), p1.y(), 1.8);
  Eigen::Vector3f test2(P0);
  test2.z() = 1.8;

//  publish_topics_->robotPositionsMsg(test);
//  publish_topics_->robotPositionsMsg(test2);

  int count = 0;

  float min_angle = 1000.0;
  float max_angle = 0.0;

  std::vector<Eigen::Vector3i> uvox_tmp;
  Eigen::Vector3f V0;
  for (int zz = dimensions_.z()-1; zz >(-1) ; zz--) {
    for (int yy = 0; yy < dimensions_.y(); ++yy){
      for(int xx = 0; xx < dimensions_.x(); ++xx){
        if( grid[zz][yy][xx].state == UNKNOWN )
        {
          Eigen::Vector3i vox(xx, yy, zz);
          V0 = voxelToCartesian(vox);

          Eigen::Vector2f p2d(V0.x(), V0.y());

         // Eigen::Vector2f plo = p2d-p2;
         // float angle = acos(normal.dot(plo.normalized()));

          Eigen::Vector2f plo = p2d-p2;
          Eigen::Vector2f pla = p1-p2;

          plo = plo.normalized();
          pla = pla.normalized();

          float dot_pro = plo.dot(pla);

          if(dot_pro > 1.0)
          {
            float diff = dot_pro - 1.0;
            dot_pro = dot_pro - (2*diff);
          }
          if(dot_pro < (-1.0) )
          {
            float diff = dot_pro + 1.0;
            dot_pro = dot_pro - (2*diff);
          }

          float angle = acos(dot_pro);
          angle = angle * 180.0 / M_PI;

          /*
          if(angle > max_angle)
            max_angle = angle;
          if(angle < min_angle)
            min_angle = angle;

*/
/*
          if(zz == 64 && yy < 10  && xx == 128)
          {
            publish_topics_->robotPositionsMsg(V0);
               std::cout << "p2d" << std::endl;
               std::cout << p2d << std::endl;
               std::cout << "p0" << std::endl;
               std::cout << p2 << std::endl;
               std::cout << "p1" << std::endl;
               std::cout << p1 << std::endl;
               std::cout << "p2d-p0" << std::endl;
               std::cout << plo << std::endl;
               std::cout << "p1-p0" << std::endl;
               std::cout << pla << std::endl;
               std::cout << "dot" << std::endl;
               std::cout << plo.dot(pla) << std::endl;
            ROS_INFO("angle: %f", angle);
          }
*/

          float dist = eulcideanDistance(P0, V0);

         // if(angle < fov && V0.x() > P0.x()+0.2 && dist > 0.4)
          if(angle < fov && dist > 0.4  )
          {
            uvox_tmp.push_back(vox);
          }

          /*
          else
          {
            if(count < 10)
            {
              count++;
           //   std::cout << p2d << std::endl;
           //   std::cout << p2 << std::endl;
           //   std::cout << p1 << std::endl;
           //   ROS_INFO("angle: %f", angle);

              publish_topics_->robotPositionsMsg(V0);
            }
            */
          //}
        }
      }
    }
  }

  //std::cout << "-------- min angle: " << min_angle << std::endl;
  //std::cout << "-------- max angle: " << max_angle << std::endl;

/*
  normal = computeNormal(p1,p2,-fov);

  Eigen::Vector3f test2(normal.x(), normal.y(), 0.0);
  publish_topics_->robotPositionsMsg(test2+P0);

  std::vector<Eigen::Vector3i> uvox_tmp2;
  for( int vox=(int)uvox_tmp.size()-1; vox>(-1); vox--)
  {
    const Eigen::Vector3i v(uvox_tmp[vox].x(),uvox_tmp[vox].y(),uvox_tmp[vox].z());
    if( grid[v.z()][v.y()][v.x()].state == UNKNOWN )
    {
      //Eigen::Vector3i vox(xx, yy, zz);
      V0 = voxelToCartesian(v);

      Eigen::Vector2f p2d(V0.x(), V0.y());

      Eigen::Vector2f plo = p2d-p2;
      float angle = acos(normal.dot(plo.normalized()));

      if(angle > fov)
      {
        uvox_tmp2.push_back(v);
      }
    }
  }
*/

/*
  // vertical
  std::vector<Eigen::Vector3i> uvox_tmp3;
  for( int vox=(int)uvox_tmp.size()-1; vox>(-1); vox--)
  {
    const Eigen::Vector3i v(uvox_tmp[vox].x(),uvox_tmp[vox].y(),uvox_tmp[vox].z());
    if( grid[v.z()][v.y()][v.x()].state == UNKNOWN )
    {
      //Eigen::Vector3i vox(xx, yy, zz);
      V0 = voxelToCartesian(v);

      float dist = eulcideanDistance(P0, V0);
      if(V0.x() > P0.x()+0.2 && dist > 0.4)
      {
        uvox_tmp3.push_back(v);
      }
    }
  }
*/
  return uvox_tmp;
}

Eigen::Vector2f OccupancyGrid::computeNormal(Eigen::Vector2f p1, Eigen::Vector2f p2, float fov)
{
  float theta = fov * M_PI/180;
  Eigen::Matrix2f R;
  R << cos(theta), -sin(theta),
       sin(theta),  cos(theta);

  Eigen::Vector2f p2f(0.0,0.0);
  p2f = R*(p1-p2) + p2;
  Eigen::Vector2f p(p2f.x(), p2f.y());

  Eigen::Vector2f pp = p-p2;
//  Eigen::Vector2f normal = pp.unitOrthogonal();
  Eigen::Vector2f normal = pp.normalized();

  return normal;
}

float OccupancyGrid::eulcideanDistance(const Eigen::Vector3f p1, const Eigen::Vector3f p2)
{
  float dist;

  dist = sqrt( pow(p1.x() - p2.x(), 2) +
               pow(p1.y() - p2.y(), 2) +
               pow(p1.z() - p2.z(), 2) );

  return dist;

}



double OccupancyGrid::computeExpectation(array_type &grid)
{
  double expectaion = 0.0;

  std::vector<Eigen::Vector3i> uvox = getVisibleVoxels(grid);

  for(unsigned int i=0; i<uvox.size(); i++)
  {
    int x,y,z;
    x = uvox[i].x();
    y = uvox[i].y();
    z = uvox[i].z();

    double p = grid[z][y][x].observation_prob;

    expectaion += p;
  }

  expectaion = expectaion * (-1);
  return expectaion;
}

std::vector<Eigen::Vector3i> OccupancyGrid::getVisibleVoxels(array_type &grid)
{
  std::vector<Eigen::Vector3i> uvox;

  for (int zz = dimensions_.z()-1; zz >(-1) ; zz--) {
    for (int yy = 0; yy < dimensions_.y(); ++yy){
      for(int xx = 0; xx < dimensions_.x(); ++xx){
        if( grid[zz][yy][xx].observation_prob > 0.0 )
        {
          Eigen::Vector3i vox(xx, yy, zz);
          uvox.push_back(vox);
        }
      }
    }
  }
  return uvox;
}


void OccupancyGrid::rayTraversal( const Eigen::Vector3f P0, std::vector<Eigen::Vector3i> &uvox, array_type &vgrid, bool real)
{
  double start = clock();

  const int w = dimensions_.y()/2;
  const int h = dimensions_.z()/2;
  const int d = dimensions_.x()/2;;

  // Point on plane
  Eigen::Vector3f V0;
  // Ray end point
  Eigen::Vector3f P1;
  // intersection point
  Eigen::Vector3f I;

  bool obstacle = false;
  bool enclosed_voxel = false;

  int unknown_count = 0;
  int count = 0;

  // cases
  // c = 1 ; bottom
  // c = 2 ; top
  // c = 3 ; right
  // c = 4 ; left
  int c = 0;

  std::vector<std::vector <Eigen::Vector3i> > rays;

  // estimate relative sampling position to the voxel grid
  // and initialize variables
  Eigen::Vector3i sp = cartesianToVoxel(P0);

  if( sp.x() <= 0 )
    c = 1;
  else if( sp.x() >= dimensions_.x() )
    c = 2;
  else if( sp.y() <= 0 && sp.x() >= 0 && sp.x() <= dimensions_.x())
    c = 3;
  else if( sp.y() >= dimensions_.y() && sp.x() >= 0 && sp.x() <= dimensions_.x())
    c = 4;


  std::cerr << "case: "<< c << "   " << " x: " << sp.x() << " y: " << sp.y() << " z: " << sp.z() << std::endl;

  c = 4;

  if( c==1 ){
    // plane normal
    Eigen::Vector3f n(-1.0, 0.0, 0.0);
    float theta = rot_angle_;
    float xr = n.x()*cos(theta)-n.y()*sin(theta);
    float yr = n.x()*sin(theta)+n.y()*cos(theta);
    n.x() = xr;
    n.y() = yr;
    n.normalize();

    for( int vox=(int)uvox.size()-1; vox>(-1); vox--)
    {
      const Eigen::Vector3i v(uvox[vox].x(),uvox[vox].y(),uvox[vox].z());
      P1 = voxelToCartesian(v);

      unknown_count = 0;
      count = 0;

      for( int depth2=uvox[vox].x(); depth2>(-1); depth2--)
      {
        const Eigen::Vector3i vox(depth2, w, h);
        V0 = voxelToCartesian(vox);

        I = intersect3DLinePlane(P0, P1, V0, n);

        Eigen::Vector3i vcoord = cartesianToVoxel(I);

        if(vcoord.z() < dimensions_.z() && vcoord.z() >= 0 &&
           vcoord.y() < dimensions_.y() && vcoord.y() >= 0 &&
           vcoord.x() < dimensions_.x() && vcoord.x() >= 0){

          if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].state == UNKNOWN)
            unknown_count++;

          if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].enclosed_prob < 1.0)
            enclosed_voxel = true;

          count++;

          if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].state == OCCUPIED){
            obstacle = true;
            break;
          }
        }else{
          break;
        }
      }
      if(!obstacle){
        double prob = 0.0;

        if(real)
        {

          prob = 0.0;
          vgrid[v.z()][v.y()][v.x()].state = FREE;
          vgrid[v.z()][v.y()][v.x()].p = 0.39;
        }else{
          prob = compute_expectation((float)unknown_count);
          if(enclosed_voxel)
           prob = prob * ENCLOSED_PROB;

        }

        //if(prob > 0.5)
        //  prob = 0.5;

        vgrid[v.z()][v.y()][v.x()].observation_prob = prob;
        enclosed_voxel = false;
      }else{
        obstacle = false;
        vgrid[v.z()][v.y()][v.x()].observation_prob = 0.0;
      }
    }
  }
  else if( c==2 ){
    // plane normal
    Eigen::Vector3f n(1.0, 0.0, 0.0);
    float theta = rot_angle_;
    float xr = n.x()*cos(theta)-n.y()*sin(theta);
    float yr = n.x()*sin(theta)+n.y()*cos(theta);
    n.x() = xr;
    n.y() = yr;
    n.normalize();

    for( int vox=(int)uvox.size()-1; vox>(-1); vox--)
    {
      Eigen::Vector3i v(uvox[vox].x(),uvox[vox].y(),uvox[vox].z());
      P1 = voxelToCartesian(v);

      unknown_count = 0;
      count = 0;

      std::vector<Eigen::Vector3i> ray;
      //voxel_count = (float)uvox[vox].x();
      for( int depth2=uvox[vox].x();depth2<dimensions_.x(); depth2++)
      {
            Eigen::Vector3i vox(depth2, w, h);
            V0 = voxelToCartesian(vox);

            I = intersect3DLinePlane(P0, P1, V0, n);

            Eigen::Vector3i vcoord = cartesianToVoxel(I);

            if(vcoord.z() < dimensions_.z() && vcoord.z() >= 0 &&
               vcoord.y() < dimensions_.y() && vcoord.y() >= 0 &&
               vcoord.x() < dimensions_.x() && vcoord.x() >= 0) {

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].state == UNKNOWN)
                unknown_count++;

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].enclosed_prob < 1.0)
                enclosed_voxel = true;

              count++;

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].state == OCCUPIED){
                obstacle = true;
                break;
              }
            }else{
              break;
            }
         }
          if(!obstacle){
            double prob = 0.0;


            if(real)
            {
              prob = 0.0;
              vgrid[v.z()][v.y()][v.x()].state = FREE;
              vgrid[v.z()][v.y()][v.x()].p = 0.39;
            }else{

              prob = compute_expectation((float)unknown_count);
              if(enclosed_voxel)
               prob = prob * ENCLOSED_PROB;

            }

          //  if(prob > 0.5)
          //    prob = 0.5;

            vgrid[v.z()][v.y()][v.x()].observation_prob = prob;
            enclosed_voxel = false;
          }else{
            obstacle = false;
            vgrid[v.z()][v.y()][v.x()].observation_prob = 0.0;
          }
        }
      }
  else if( c==3 ){
    // plane normal
    Eigen::Vector3f n(0.0, 1.0, 0.0);
    float theta = rot_angle_;
    float xr = n.x()*cos(theta)-n.y()*sin(theta);
    float yr = n.x()*sin(theta)+n.y()*cos(theta);
    n.x() = xr;
    n.y() = yr;
    n.normalize();

    for( int vox=(int)uvox.size()-1; vox>(-1); vox--)
    {
      Eigen::Vector3i v(uvox[vox].x(),uvox[vox].y(),uvox[vox].z());
      P1 = voxelToCartesian(v);

      unknown_count = 0;
      count = 0;

      std::vector<Eigen::Vector3i> ray;
      for( int width2=uvox[vox].y(); width2>(-1); width2--)
      {

            Eigen::Vector3i vox(d, width2, h);
            V0 = voxelToCartesian(vox);

            I = intersect3DLinePlane(P0, P1, V0, n);

            Eigen::Vector3i vcoord = cartesianToVoxel(I);

            if(vcoord.z() < dimensions_.z() && vcoord.z() >= 0 &&
               vcoord.y() < dimensions_.y() && vcoord.y() >= 0 &&
               vcoord.x() < dimensions_.x() && vcoord.x() >= 0) {

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].state == UNKNOWN)
                unknown_count++;

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].enclosed_prob < 1.0)
                enclosed_voxel = true;

              count++;

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].state == OCCUPIED){
                obstacle = true;
                break;
              }
            }else{
              break;
            }
          }
        if(!obstacle){
          double prob = 0.0;


          if(real)
           {
             prob = 0.0;
             vgrid[v.z()][v.y()][v.x()].state = FREE;
             vgrid[v.z()][v.y()][v.x()].p = 0.39;
           }else{

             prob = compute_expectation((float)unknown_count);
             if(enclosed_voxel)
              prob = prob * ENCLOSED_PROB;

           }

        //  if(prob > 0.5)
        //    prob = 0.5;

          vgrid[v.z()][v.y()][v.x()].observation_prob = prob;
          enclosed_voxel = false;
        }else{
          obstacle = false;
          vgrid[v.z()][v.y()][v.x()].observation_prob = 0.0;
        }
        }
      }

  else if( c==4 ){
    // plane normal
    Eigen::Vector3f n(0.0, 1.0, 0.0);
    float theta = rot_angle_;
    float xr = n.x()*cos(theta)-n.y()*sin(theta);
    float yr = n.x()*sin(theta)+n.y()*cos(theta);
    n.x() = xr;
    n.y() = yr;
    n.normalize();

    for( int vox=(int)uvox.size()-1; vox>(-1); vox--)
    {
      Eigen::Vector3i v(uvox[vox].x(),uvox[vox].y(),uvox[vox].z());
      P1 = voxelToCartesian(v);

      unknown_count = 0;
      count = 0;

      std::vector<Eigen::Vector3i> ray;
      for( int width2=uvox[vox].y(); width2<dimensions_.y(); width2++)
      {
            Eigen::Vector3i vox(d, width2, h);
            V0 = voxelToCartesian(vox);

            I = intersect3DLinePlane(P0, P1, V0, n);

            Eigen::Vector3i vcoord = cartesianToVoxel(I);

            if(vcoord.z() < dimensions_.z() && vcoord.z() >= 0 &&
               vcoord.y() < dimensions_.y() && vcoord.y() >= 0 &&
               vcoord.x() < dimensions_.x() && vcoord.x() >= 0) {

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].state == UNKNOWN)
                unknown_count++;

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].enclosed_prob < 1.0)
                enclosed_voxel = true;

              count++;

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].state == OCCUPIED){
                obstacle = true;
                break;
              }
            }else{
              break;
            }
          }
        if(!obstacle){
          double prob = 0.0;


          if(real)
           {
             prob = 0.0;
             vgrid[v.z()][v.y()][v.x()].state = FREE;
             vgrid[v.z()][v.y()][v.x()].p = 0.39;
           }else{

            prob = compute_expectation((float)unknown_count);
            if(enclosed_voxel)
             prob = prob * ENCLOSED_PROB;

           }

         // if(prob > 0.5)
        //   prob = 0.5;

          vgrid[v.z()][v.y()][v.x()].observation_prob = prob;
          enclosed_voxel = false;
        }else{
          obstacle = false;
          vgrid[v.z()][v.y()][v.x()].observation_prob = 0.0;
        }
        }
      }



  double finish = clock();

  std::cerr << "Ray traversal DONE in: "<< ( (finish - start)/CLOCKS_PER_SEC ) << std::endl;


 // analyzeRay(rays, vgrid);


  std::cerr << "Ray analyze: DONE" << std::endl;



}

Eigen::Vector3f OccupancyGrid::intersect3DLinePlane(const Eigen::Vector3f &P0, Eigen::Vector3f &P1,
                                           Eigen::Vector3f &V0, const Eigen::Vector3f &n)
{
  Eigen::Vector3f u = P1 - P0;
  Eigen::Vector3f w = P0 - V0;

  double D = n.dot(u);
  double N = -n.dot(w);

/*
  if (fabs(D) < 0.00000001) {          // segment is parallel to plane
        if (N == 0)                     // segment lies in plane
            ROS_INFO("segment is parallel to plane - segment lies in plane");
        else
          ROS_INFO("segment is parallel to plane - no intersection");       // no intersection
    }
*/

  double SI = N/D;

 // if (SI < 0.0 || SI > 1.0)
 //   ROS_INFO("no intersection: %f", SI);
  //else
   // ROS_INFO("intersection");

  Eigen::Vector3f I = P0 + SI * u;

  return I;
}

std::vector<Eigen::Vector3i>
OccupancyGrid::growRegion(Eigen::Vector3i vox, int v_size)
{
  std::vector<Eigen::Vector3i> volume;
  Eigen::Vector3i v_start = vox;
  Eigen::Vector3i v_end = vox;
  bool fit_volume = true;

  // set the new start and end point of the bbox
  int size = floor(v_size / 2);
  v_start.x() = v_start.x() - size ;
  v_start.y() = v_start.y() - size ;
  v_start.z() = v_start.z() - size ;
  v_end.x() = v_end.x() + size ;
  v_end.y() = v_end.y() + size ;
  v_end.z() = v_end.z() + size ;

  for (int zz = v_start.z(); zz < v_end.z()+1; ++zz) {
    for (int yy = v_start.y(); yy < v_end.y()+1; ++yy){
      for(int xx = v_start.x(); xx < v_end.x()+1; ++xx){
        if(xx < 0 || yy < 0 || zz < 0)
        {
         // std::cout << "Out of Bound" << std::endl;
          fit_volume = false;
          break;
        }
        if(xx > dimensions_.x()-1 || yy > dimensions_.y()-1 || zz > dimensions_.z()-1)
        {
         // std::cout << "Out of Bound" << std::endl;
          fit_volume = false;
          break;
        }
        if(occupancy_grid_[zz][yy][xx].state != UNKNOWN )
        {
         // std::cout << "not UNKNOWN" << std::endl;
          fit_volume = false;
        }

        Eigen::Vector3i v(xx, yy, zz);
        volume.push_back(v);

      }
    }
  }

  if(fit_volume)
  {
    std::cout << "Volume Fit: "<< vox.x() <<" " << vox.y() << " " << vox.z() << std::endl;
    return volume;
  }

  volume.clear();
  return volume;
}

std::vector<std::vector<Eigen::Vector3i> > OccupancyGrid::findVolumes(int v_size)
{
  unsigned int n_samples = 2;
  std::vector<std::vector<Eigen::Vector3i> > volumes;

  // get the unknown voxels from the grid
  std::vector<Eigen::Vector3i> uvox;
  uvox = getUnknownVoxels(occupancy_grid_);

  std::cout << uvox.size() << std::endl;

 // for(unsigned int s=0; s<n_samples; s++)
  for(unsigned int s=0; s< 272722-1; s++)
  {
    // sample a random unknown voxel
    int nHigh = uvox.size() - 1;
    int nLow = 0;
    int index = (rand() % (nHigh - nLow + 1)) + nLow;

    //Eigen::Vector3i vox = uvox[index];
    Eigen::Vector3i vox = uvox[s];

    //Eigen::Vector3i vox(104,56,39);
    // region growing in the grid given a voxel as centroid
    std::vector<Eigen::Vector3i> volume;
    volume = growRegion(vox, v_size);

    if(volume.size() != 0)
      std::cout << volume.size() << std::endl;

  }

  return volumes;
}

void OccupancyGrid::wall()
{
 // for (int zz = 0; zz < dimensions_.z(); ++zz) {
 //   for (int yy = 128; yy < dimensions_.y(); ++yy){
 //     for(int xx = 0; xx < 1; ++xx){
 //       occupancy_grid_[zz][yy][xx].state = OCCUPIED;
 //       occupancy_grid_[zz][yy][xx].p = 0.9;

  //    }
  //  }
  //}

         occupancy_grid_[10][128][0].state = OCCUPIED;
         occupancy_grid_[10][128][0].p = 0.9;
}

void OccupancyGrid::colorUnknownVoxels(std::vector<Eigen::Vector3i> uvox)

{
  for( int vox=(int)uvox.size()-1; vox>(-1); vox--)
  {
    const Eigen::Vector3i v(uvox[vox].x(),uvox[vox].y(),uvox[vox].z());

         occupancy_grid_[v.z()][v.y()][v.x()].state = OCCUPIED;
         occupancy_grid_[v.z()][v.y()][v.x()].p = 0.9;

   }

}


void OccupancyGrid::voxelGridMsg()
{

  visualization_msgs::Marker occupancy_grid_msg;
  visualization_msgs::Marker voxel_grid_unknown_msg;
  visualization_msgs::Marker entropy_msg_;

  visualization_msgs::Marker voxel_grid_free_msg;

  occupancy_grid_msg.header.frame_id = "/base_link";
  occupancy_grid_msg.header.stamp = ros::Time::now();
  occupancy_grid_msg.ns = "Occupancy_grid > 0.6";
  occupancy_grid_msg.id = 0;
  occupancy_grid_msg.type = visualization_msgs::Marker::POINTS;
  occupancy_grid_msg.action = visualization_msgs::Marker::ADD;

  voxel_grid_unknown_msg.header.frame_id = "/base_link";
  voxel_grid_unknown_msg.header.stamp = ros::Time::now();
  voxel_grid_unknown_msg.ns = "Unknown";
  voxel_grid_unknown_msg.id = 0;
  voxel_grid_unknown_msg.type = visualization_msgs::Marker::POINTS;
  voxel_grid_unknown_msg.action = visualization_msgs::Marker::ADD;

  voxel_grid_free_msg.header.frame_id = "/base_link";
  voxel_grid_free_msg.header.stamp = ros::Time::now();
  voxel_grid_free_msg.ns = "Free";
  voxel_grid_free_msg.id = 0;
  voxel_grid_free_msg.type = visualization_msgs::Marker::POINTS;
  voxel_grid_free_msg.action = visualization_msgs::Marker::ADD;


  entropy_msg_.header.frame_id = "/base_link";
  entropy_msg_.header.stamp = ros::Time::now();
  entropy_msg_.ns = "Entropy";
  entropy_msg_.id = 0;
  entropy_msg_.type = visualization_msgs::Marker::POINTS;
  entropy_msg_.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point p;

  for (int zz = 0; zz < dimensions_.z(); ++zz) {
    for (int yy = 0; yy < dimensions_.y(); ++yy){
      for(int xx = 0; xx < dimensions_.x(); ++xx){
        const Eigen::Vector3i vox(xx, yy, zz);
        Eigen::Vector3f coord;
        coord = voxelToCartesian(vox);

        p.x = coord.x();
        p.y = coord.y();
        p.z = coord.z();

        if( occupancy_grid_[zz][yy][xx].state == OCCUPIED )
        {
          std_msgs::ColorRGBA color;
          color.r = 1.0;
          color.g = (1.0/0.4)*(1-occupancy_grid_[zz][yy][xx].p);
          color.b = 0.0;
          color.a = 1.0;
          occupancy_grid_msg.colors.push_back(color);
          occupancy_grid_msg.points.push_back(p);
        }

        if( occupancy_grid_[zz][yy][xx].state == UNKNOWN )
        {
          voxel_grid_unknown_msg.points.push_back(p);
        }

        if( occupancy_grid_[zz][yy][xx].state == FREE )
        {
          voxel_grid_free_msg.points.push_back(p);
        }

        if( occupancy_grid_[zz][yy][xx].observation_prob > 0.0 )
        {
          std_msgs::ColorRGBA color;
          color.r = 1.0;
          color.g =  occupancy_grid_[zz][yy][xx].observation_prob;
          color.b = 0.0;
          color.a = 1.0;
          entropy_msg_.colors.push_back(color);
          entropy_msg_.points.push_back(p);
        }

      }
    }
  }

  float scale_x = 0.011946;
  float scale_y = 0.005746;
  float scale_z = 0.004642;

  occupancy_grid_msg.scale.x = scale_x;
  occupancy_grid_msg.scale.y = scale_y;
  occupancy_grid_msg.scale.z = scale_z;
  voxel_grid_unknown_msg.scale.x = scale_x;
  voxel_grid_unknown_msg.scale.y = scale_y;
  voxel_grid_unknown_msg.scale.z = scale_z;
  entropy_msg_.scale.x = scale_x;
  entropy_msg_.scale.y = scale_y;
  entropy_msg_.scale.z = scale_z;
  voxel_grid_free_msg.scale.x = scale_x;
  voxel_grid_free_msg.scale.y = scale_y;
  voxel_grid_free_msg.scale.z = scale_z;

  voxel_grid_unknown_msg.color.r = 1.0;
  voxel_grid_unknown_msg.color.g = 1.0;
  voxel_grid_unknown_msg.color.b = 0.0;
  voxel_grid_unknown_msg.color.a = 1.0;

  voxel_grid_free_msg.color.r = 0.0;
  voxel_grid_free_msg.color.g = 1.0;
  voxel_grid_free_msg.color.b = 0.0;
  voxel_grid_free_msg.color.a = 1.0;

  occupancy_grid_msg.color.a = 1.0;
  entropy_msg_.color.a = 1.0;

  publish_topics_->newVisualizationMarker(occupancy_grid_msg);
  publish_topics_->newVisualizationMarker(voxel_grid_unknown_msg);
  publish_topics_->newVisualizationMarker(entropy_msg_);
  publish_topics_->newVisualizationMarker(voxel_grid_free_msg);

}

void OccupancyGrid::mrfVisul()
{
  visualization_msgs::Marker mrf_msg;

  mrf_msg.header.frame_id = "/base_link";
  mrf_msg.header.stamp = ros::Time::now();
  mrf_msg.ns = "mrf";
  mrf_msg.id = 0;
  mrf_msg.type = visualization_msgs::Marker::POINTS;
  mrf_msg.action = visualization_msgs::Marker::ADD;

  for(int i=0; i<(int)mrfs_.size(); i++)
  {
    double h, w, d;
    const Eigen::Vector3f P0(objd_[i](0), objd_[i](1), objd_[i](2));
    const Eigen::Vector3f P1(objd_[i](3), objd_[i](4), objd_[i](5));
    d = (cartesianToVoxel(P1).x() - cartesianToVoxel(P0).x()) +1;
    w = (cartesianToVoxel(P1).y() - cartesianToVoxel(P0).y()) +1;
    h = (cartesianToVoxel(P1).z() - cartesianToVoxel(P0).z()) +1;

    geometry_msgs::Point p;
    for (int zz = 0; zz < h; ++zz) {
      for (int yy = 0; yy < w; ++yy){
        for(int xx = 0; xx < d; ++xx){
          int z = cartesianToVoxel(P0).z() + zz;
          int y = cartesianToVoxel(P0).y() + yy;
          int x = cartesianToVoxel(P0).x() + xx;

          Eigen::Vector3i vox(x, y, z);
          Eigen::Vector3f coord;
          coord = voxelToCartesian(vox);

          p.x = coord.x();
          p.y = coord.y();
          p.z = coord.z();


          if( mrfs_[i][zz][yy][xx].state == OCCUPIED )
            mrf_msg.points.push_back(p);

        }
      }
    }


  }

  float scale_x = 0.011946;
  float scale_y = 0.005746;
  float scale_z = 0.004642;

  mrf_msg.scale.x = scale_x;
  mrf_msg.scale.y = scale_y;
  mrf_msg.scale.z = scale_z;

  mrf_msg.color.r = 0.0;
  mrf_msg.color.g = 1.0;
  mrf_msg.color.b = 1.0;
  mrf_msg.color.a = 1.0;

  publish_topics_->newVisualizationMarker(mrf_msg);

}




