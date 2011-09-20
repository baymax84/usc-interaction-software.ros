/*
 * voxel_grid.cpp
 *
 *  Created on: Dec 5, 2010
 *      Author: Christian Potthast
 */

#include <nbv_arm_planning/voxel_grid.h>

VoxelGrid::VoxelGrid(Eigen::Vector3f box, Eigen::Vector3i dim,
                     Eigen::Vector4f centroid, std::vector<Eigen::Vector3f> table_top_bbx,
                     PublishTopics& publish_topics)
{
  publish_topics_ = &publish_topics;

  dimensions_ = Eigen::Vector3i(dim.x(), dim.y(), dim.z());
  bbx_ = Eigen::Vector3f(box.x(), box.y(), box.z());
  centroid_ = centroid;
  voxel_ = Eigen::Vector3f(box.x() / dim.x(), box.y() / dim.y(), box.z() / dim.z());
  centroid_voxel_ = Eigen::Vector3f( (dim.x()/2) , (dim.y()/2), (dim.z()/2) );

  voxel_grid_.resize(boost::extents[dimensions_.z()][dimensions_.y()][dimensions_.x()]);
  virtual_grid_.resize(boost::extents[dimensions_.z()][dimensions_.y()][dimensions_.x()]);

  // initialize all fields with 0.5 (unknown);
  voxel_t v;
  for (int zz = 0; zz < dimensions_.z(); ++zz) {
    for (int yy = 0; yy < dimensions_.y(); ++yy){
      for(int xx = 0; xx < dimensions_.x(); ++xx){
        v.value = UNKNOWN;
        v.observation_prob = 0.5;
        v.enclosed_prob = 1.0;
        voxel_grid_[zz][yy][xx] = v;
      }
    }
  }

  publish_topics_->voxelGridBbxMsg(table_top_bbx, bbx_);
}

void VoxelGrid::setGridPoints(pcl::PointCloud<PointRGB>::ConstPtr &point)
{

  if((int)cloud_.points.size() < 1)
    cloud_ = *point;
  else
    cloud_ += *point;


  for( int i=0; i < (int)point->points.size(); i++){
    Eigen::Vector3f p(point->points[i].x, point->points[i].y, point->points[i].z);
    Eigen::Vector3i coord = cartesianToVoxel(p);

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

    if(coord.y() < 0)
    {
      coord.y() = 0;
    }

    //ROS_INFO("%d %d %d", coord.x(), coord.y(), coord.z());


    voxel_grid_[coord.z()][coord.y()][coord.x()].value = OCCUPIED;
    voxel_grid_[coord.z()][coord.y()][coord.x()].observation_prob = 0.0;

  }

}

void VoxelGrid::voxelGridError()
{
  double avg_err_x = 0.0;
  double avg_err_y = 0.0;
  double avg_err_z = 0.0;

  for( int i=0; i < (int)cloud_.points.size(); i++){
    Eigen::Vector3f p(cloud_.points[i].x, cloud_.points[i].y, cloud_.points[i].z);
    Eigen::Vector3i vox = cartesianToVoxel(p);
    Eigen::Vector3f coord = voxelToCartesian(vox);

    double avg_err_x_tmp = fabs( coord.x() - cloud_.points[i].x) ;
    double avg_err_y_tmp = fabs( coord.y() - cloud_.points[i].y );
    double avg_err_z_tmp = fabs( coord.z() - cloud_.points[i].z );

    avg_err_x = avg_err_x + avg_err_x_tmp;
    avg_err_y = avg_err_y + avg_err_y_tmp;
    avg_err_z = avg_err_z + avg_err_z_tmp;
  }

  avg_err_x = avg_err_x/(int)cloud_.points.size();
  avg_err_y = avg_err_y/(int)cloud_.points.size();
  avg_err_z = avg_err_z/(int)cloud_.points.size();

  ROS_INFO("Voxel Grid: average error: x: %f y: %f z: %f", avg_err_x, avg_err_y, avg_err_z);
}


void VoxelGrid::createMRFs()
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
          if( voxel_grid_[z][y][x].value == FREE || voxel_grid_[z][y][x].value == UNKNOWN){
            v.value = -1;
            v.observation_prob = 0.0;
            mrf[zz][yy][xx] = v;
          }else if( voxel_grid_[z][y][x].value == OCCUPIED ){
            v.value = OCCUPIED;
            mrf[zz][yy][xx] = v;
          }
        }
      }
    }

    mrfs_.push_back(mrf);

  }
}

void VoxelGrid::MRFToVG()
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

          if(mrfs_[i][zz][yy][xx].value == OCCUPIED){
            voxel_grid_[z][y][x].value = mrfs_[i][zz][yy][xx].value;
            voxel_grid_[z][y][x].observation_prob = mrfs_[i][zz][yy][xx].observation_prob;
          }else{
            if(voxel_grid_[z][y][x].value == UNKNOWN)
              voxel_grid_[z][y][x].enclosed_prob = ENCLOSED_PROB;
          }

        }
      }
    }
  }

}


void VoxelGrid::ICM(array_type &mrf, array_type &hl, Eigen::Vector3i &dim)
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
        int v = hl[zz][yy][xx].value;

        if(mrf[zz][yy][xx].value != OCCUPIED){

        // extract the neighbors from the fixed voxel
        nb.clear();
        if( xx < d-1){
          nb.push_back(hl[zz][yy][xx+1].value);
        }else
          nb.push_back(0);
        if( xx > 0){
          nb.push_back(hl[zz][yy][xx-1].value);
        }else
          nb.push_back(0);
        if( yy < w-1){
          nb.push_back(hl[zz][yy+1][xx].value);
        }else
          nb.push_back(0);
        if( yy > 0){
          nb.push_back(hl[zz][yy-1][xx].value);
        }else
          nb.push_back(0);
        if( zz < h-1){
          nb.push_back(hl[zz+1][yy][xx].value);
        }else
          nb.push_back(0);
        if( zz > 0){
          nb.push_back(hl[zz-1][yy][xx].value);
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
          hl[zz][yy][xx].value = v_2;
          converge = false;

          energy += energy_2;

       //   std::cerr << "change: "<< v_2 << std::endl;
        }else{
          energy += energy_1;

        }


        energy += -2.1*(hl[zz][yy][xx].value * mrf[zz][yy][xx].value);



      }
    }
  }

  }

  iter++;
  std::cerr << iter << "    energy: "<< energy <<  std::endl;

}

  std::cerr << "MRF: DONE" << "    Iterations: " << iter << std::endl;

}

void VoxelGrid::evalMrf()
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

/*
void VoxelGrid::rayTraversal( const Eigen::Vector3f P0)
{
  double start = clock();

  const int w = dimensions_.y()/2;
  const int h = dimensions_.z()/2;
  const int d = dimensions_.x()/2;

  // Point on plane
  Eigen::Vector3f V0;
  // Ray end point
  Eigen::Vector3f P1;
  // intersection point
  Eigen::Vector3f I;

  // cases
  // c = 1 ; bottom
  // c = 2 ; top
  // c = 3 ; left
  // c = 4 ; right
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


  std::cerr << "sp: "<< "case: "<< c << "   " << " x: " << sp.x() << " y: " << sp.y() << " z: " << sp.z() << std::endl;

  if( c==1 ){
    // plane normal
    Eigen::Vector3f n(-1.0, 0.0, 0.0);
    float theta = rot_angle_;
    float xr = n.x()*cos(theta)-n.y()*sin(theta);
    float yr = n.x()*sin(theta)+n.y()*cos(theta);
    n.x() = xr;
    n.y() = yr;
    n.normalize();


    for (int zz = dimensions_.z()-1; zz >(-1) ; zz--) {
      for (int yy = 0; yy < dimensions_.y(); ++yy){
        for(int xx = 0; xx < dimensions_.x(); ++xx){
          Eigen::Vector3i v(xx,yy,zz);
          P1 = voxelToCartesian(v);

          std::vector<Eigen::Vector3i> ray;
          for( int depth2=xx; depth2>(-1); depth2--)
          {
            Eigen::Vector3i vox(depth2, w, h);
            V0 = voxelToCartesian(vox);

            I = intersect3DLinePlane(P0, P1, V0, n);

            Eigen::Vector3i vcoord = cartesianToVoxel(I);

            if(vcoord.z() < dimensions_.z() && vcoord.z() >= 0 &&
               vcoord.y() < dimensions_.y() && vcoord.y() >= 0 &&
               vcoord.x() < dimensions_.x() && vcoord.x() >= 0)
              ray.push_back(vcoord);
          }
          rays.push_back(ray);
        }
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

    for (int zz = dimensions_.z()-1; zz >(-1) ; zz--) {
      for (int yy = 0; yy < dimensions_.y(); ++yy){
        for(int xx = 0; xx < dimensions_.x(); ++xx){
          Eigen::Vector3i v(xx,yy,zz);
          P1 = voxelToCartesian(v);

          std::vector<Eigen::Vector3i> ray;
          for( int depth2=xx; depth2<dimensions_.x(); depth2++)
          {
            Eigen::Vector3i vox(depth2, w, h);
            V0 = voxelToCartesian(vox);

            I = intersect3DLinePlane(P0, P1, V0, n);

            Eigen::Vector3i vcoord = cartesianToVoxel(I);

            if(vcoord.z() < dimensions_.z() && vcoord.z() >= 0 &&
               vcoord.y() < dimensions_.y() && vcoord.y() >= 0 &&
               vcoord.x() < dimensions_.x() && vcoord.x() >= 0)
              ray.push_back(vcoord);
          }
          rays.push_back(ray);
        }
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

    for (int zz = dimensions_.z()-1; zz >(-1) ; zz--) {
      for (int yy = 0; yy < dimensions_.y(); ++yy){
        for(int xx = 0; xx < dimensions_.x(); ++xx){

          Eigen::Vector3i v(xx,yy,zz);
          P1 = voxelToCartesian(v);

          std::vector<Eigen::Vector3i> ray;
          for( int width2=yy; width2>(-1); width2--)
          {
            Eigen::Vector3i vox(d, width2, h);
            V0 = voxelToCartesian(vox);

            I = intersect3DLinePlane(P0, P1, V0, n);

            Eigen::Vector3i vcoord = cartesianToVoxel(I);

            if(vcoord.z() < dimensions_.z() && vcoord.z() >= 0 &&
               vcoord.y() < dimensions_.y() && vcoord.y() >= 0 &&
               vcoord.x() < dimensions_.x() && vcoord.x() >= 0)
              ray.push_back(vcoord);
          }
          rays.push_back(ray);
        }
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

    for (int zz = dimensions_.z()-1; zz >(-1) ; zz--) {
      for (int yy = 0; yy < dimensions_.y(); ++yy){
        for(int xx = 0; xx < dimensions_.x(); ++xx){
          Eigen::Vector3i v(xx,yy,zz);
          P1 = voxelToCartesian(v);

          std::vector<Eigen::Vector3i> ray;
          for( int width2=yy; width2<dimensions_.y(); width2++)
          {
            Eigen::Vector3i vox(d, width2, h);
            V0 = voxelToCartesian(vox);

            I = intersect3DLinePlane(P0, P1, V0, n);

            Eigen::Vector3i vcoord = cartesianToVoxel(I);

            if(vcoord.z() < dimensions_.z() && vcoord.z() >= 0 &&
               vcoord.y() < dimensions_.y() && vcoord.y() >= 0 &&
               vcoord.x() < dimensions_.x() && vcoord.x() >= 0)
              ray.push_back(vcoord);
          }
          rays.push_back(ray);
        }
      }
    }
  }



  double finish = clock();

  std::cerr << "Ray traversal DONE in: "<< ( (finish - start)/CLOCKS_PER_SEC ) << std::endl;


  analyzeRay(rays, voxel_grid_);


  std::cerr << "Ray analyze: DONE" << std::endl;

}

void VoxelGrid::rayTraversal( const Eigen::Vector3f P0, std::vector<Eigen::Vector3i> &uvox, array_type &vgrid)
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

  if( c==1 ){
    // plane normal
    Eigen::Vector3f n(-1.0, 0.0, 0.0);
    float theta = rot_angle_;
    float xr = n.x()*cos(theta)-n.y()*sin(theta);
    float yr = n.x()*sin(theta)+n.y()*cos(theta);
    n.x() = xr;
    n.y() = yr;
    n.normalize();

    //for (int vox=0; vox<(int)uvox.size(); vox++)
  //  for(long int i=(long int)rays.size() - 1; i>(-1); i--)
    for( int vox=(int)uvox.size()-1; vox>(-1); vox--)
    {
      Eigen::Vector3i v(uvox[vox].x(),uvox[vox].y(),uvox[vox].z());
      P1 = voxelToCartesian(v);

      std::vector<Eigen::Vector3i> ray;
      for( int depth2=uvox[vox].x(); depth2>(-1); depth2--)
      {

            Eigen::Vector3i vox(depth2, w, h);
            V0 = voxelToCartesian(vox);

            I = intersect3DLinePlane(P0, P1, V0, n);

            Eigen::Vector3i vcoord = cartesianToVoxel(I);

            if(vcoord.z() < dimensions_.z() && vcoord.z() >= 0 &&
               vcoord.y() < dimensions_.y() && vcoord.y() >= 0 &&
               vcoord.x() < dimensions_.x() && vcoord.x() >= 0)

              ray.push_back(vcoord);
          }
          rays.push_back(ray);
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

    for (int vox=0; vox<(int)uvox.size(); vox++)
    {
      Eigen::Vector3i v(uvox[vox].x(),uvox[vox].y(),uvox[vox].z());
      P1 = voxelToCartesian(v);

      std::vector<Eigen::Vector3i> ray;
      for( int depth2=uvox[vox].x();depth2<dimensions_.x(); depth2++)
      {
            Eigen::Vector3i vox(depth2, w, h);
            V0 = voxelToCartesian(vox);

            I = intersect3DLinePlane(P0, P1, V0, n);

            Eigen::Vector3i vcoord = cartesianToVoxel(I);

            if(vcoord.z() < dimensions_.z() && vcoord.z() >= 0 &&
               vcoord.y() < dimensions_.y() && vcoord.y() >= 0 &&
               vcoord.x() < dimensions_.x() && vcoord.x() >= 0)
              ray.push_back(vcoord);
          }
          rays.push_back(ray);
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

    for (int vox=0; vox<(int)uvox.size(); vox++)
    {
      Eigen::Vector3i v(uvox[vox].x(),uvox[vox].y(),uvox[vox].z());
      P1 = voxelToCartesian(v);

      std::vector<Eigen::Vector3i> ray;
      for( int width2=uvox[vox].y(); width2>(-1); width2--)
      {

            Eigen::Vector3i vox(d, width2, h);
            V0 = voxelToCartesian(vox);

            I = intersect3DLinePlane(P0, P1, V0, n);

            Eigen::Vector3i vcoord = cartesianToVoxel(I);

            if(vcoord.z() < dimensions_.z() && vcoord.z() >= 0 &&
               vcoord.y() < dimensions_.y() && vcoord.y() >= 0 &&
               vcoord.x() < dimensions_.x() && vcoord.x() >= 0)
              ray.push_back(vcoord);
          }
          rays.push_back(ray);
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

    for (int vox=0; vox<(int)uvox.size(); vox++)
    {
      Eigen::Vector3i v(uvox[vox].x(),uvox[vox].y(),uvox[vox].z());
      P1 = voxelToCartesian(v);

      std::vector<Eigen::Vector3i> ray;
      for( int width2=uvox[vox].y(); width2<dimensions_.y(); width2++)
      {
            Eigen::Vector3i vox(d, width2, h);
            V0 = voxelToCartesian(vox);

            I = intersect3DLinePlane(P0, P1, V0, n);

            Eigen::Vector3i vcoord = cartesianToVoxel(I);

            if(vcoord.z() < dimensions_.z() && vcoord.z() >= 0 &&
               vcoord.y() < dimensions_.y() && vcoord.y() >= 0 &&
               vcoord.x() < dimensions_.x() && vcoord.x() >= 0)
              ray.push_back(vcoord);
          }
          rays.push_back(ray);
        }
      }



  double finish = clock();

  std::cerr << "Ray traversal DONE in: "<< ( (finish - start)/CLOCKS_PER_SEC ) << std::endl;


  analyzeRay(rays, vgrid);


  std::cerr << "Ray analyze: DONE" << std::endl;



}
*/


void VoxelGrid::rayTraversal2( const Eigen::Vector3f P0, std::vector<Eigen::Vector3i> &uvox, array_type &vgrid, bool real)
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

          if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].value == UNKNOWN)
            unknown_count++;

          if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].enclosed_prob < 1.0)
            enclosed_voxel = true;


          count++;

          if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].value == OCCUPIED){
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
          vgrid[v.z()][v.y()][v.x()].value = FREE;
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

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].value == UNKNOWN)
                unknown_count++;

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].enclosed_prob < 1.0)
                enclosed_voxel = true;

              count++;

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].value == OCCUPIED){
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
              vgrid[v.z()][v.y()][v.x()].value = FREE;
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

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].value == UNKNOWN)
                unknown_count++;

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].enclosed_prob < 1.0)
                enclosed_voxel = true;

              count++;

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].value == OCCUPIED){
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
             vgrid[v.z()][v.y()][v.x()].value = FREE;
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

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].value == UNKNOWN)
                unknown_count++;

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].enclosed_prob < 1.0)
                enclosed_voxel = true;

              count++;

              if(vgrid[vcoord.z()][vcoord.y()][vcoord.x()].value == OCCUPIED){
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
             vgrid[v.z()][v.y()][v.x()].value = FREE;
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






void VoxelGrid::traversal(const Eigen::Vector3f P0, std::vector<Eigen::Vector3i> &uvox, array_type &vgrid)
{
  Eigen::Vector3f vmin;
  Eigen::Vector3f vmax;
  vmin.x() = centroid_.x() - (bbx_.x()/2);
  vmin.y() = centroid_.y() - (bbx_.y()/2);
  vmin.z() = centroid_.z() - (bbx_.z()/2);
  vmax.x() = centroid_.x() + (bbx_.x()/2);
  vmax.y() = centroid_.y() + (bbx_.y()/2);
  vmax.z() = centroid_.z() + (bbx_.z()/2);


  for(unsigned int vox_i=0; vox_i<uvox.size(); vox_i++)
  {
    std::vector<Eigen::Vector3i> ray;
    Eigen::Vector3i vox(uvox[vox_i].x(),uvox[vox_i].y(),uvox[vox_i].z());




    Eigen::Vector3f p;
    p.x() = P0.x();
    p.y() = P0.y();

    float theta = rot_angle_;
    float xr = p.x()*cos(theta)-p.y()*sin(theta);
    float yr = p.x()*sin(theta)+p.y()*cos(theta);

    Eigen::Vector3f P00(xr, yr, P0.z());


//    const Eigen::Vector3f P1 = voxelToCartesian(vox);

    Eigen::Vector3f coord;
    double x_length = (vox.x() - centroid_voxel_.x()) * voxel_.x();
    double y_length = (vox.y() - centroid_voxel_.y()) * voxel_.y();
    double z_length = (vox.z() - centroid_voxel_.z()) * voxel_.z();

    coord.x() = x_length + centroid_.x();
    coord.y() = y_length + centroid_.y();
    coord.z() = z_length + centroid_.z();

    const Eigen::Vector3f P1(coord);


    const Eigen::Vector3f raydir(P1 - P00);

    double tmin = intersect(P00, P1);

    Eigen::Vector3f start = P00 + tmin*raydir;

    publish_topics_->robotPositionsMsg(P00);
    publish_topics_->robotPositionsMsg(start);
    publish_topics_->robotPositionsMsg(P1);

    std::cout << "P0: " << P00.x() << " " << P00.y() << " " << P00.z() << std::endl;
    std::cout << "P1: " << P1.x() << " " << P1.y() << " " << P1.z() << std::endl;
    //const Eigen::Vector3i vv = cartesianToVoxel(P0);
    //std::cout << "P0: " << vv.x() << " " << vv.y() << " " << vv.z() << std::endl;

    std::cout << "start: " << start.x() << " " << start.y() << " " << start.z() << std::endl;


    Eigen::Vector3i coord2;

    int vox_x = round( ( start.x() - centroid_.x() ) /voxel_.x());
    int vox_y = round( ( start.y() - centroid_.y() ) /voxel_.y());
    int vox_z = round( ( start.z() - centroid_.z() ) /voxel_.z());

    coord2.x() = vox_x + centroid_voxel_.x();
    coord2.y() = vox_y + centroid_voxel_.y();
    coord2.z() = vox_z + centroid_voxel_.z();

    Eigen::Vector3i v = coord2;

    //std::cout << "coord2: " << coord2.x() << " " << coord2.y() << " " << coord2.z() << std::endl;

 //   const Eigen::Vector3i v = cartesianToVoxel(start);
    int x=v.x()+1;
    int y=v.y();
    int z=v.z();

    if( x == dimensions_.x() )
      x=x-1;
    if( y == dimensions_.y() )
      y=y-1;
    if( z == dimensions_.z() )
      z=z-1;

    Eigen::Vector3i tVoxel;
    int stepX, stepY, stepZ;
    if (raydir.x() >= 0)
    {
     // tVoxel.x() = x/dimensions_.x();
     // tVoxel.x() = start.x() + (voxel_.x()/2);
      tVoxel.x() = x + 1;
      stepX = 1;
    }
    else {
      tVoxel.x() = (x-1)/dimensions_.x();
      stepX = -1;
    }

    if (raydir.y() >= 0)
    {
      tVoxel.y() = y/dimensions_.y();
      stepY = 1;
    }
    else {
    //  tVoxel.y() = (y-1)/dimensions_.y();
    //  tVoxel.y() = start.y() + (voxel_.y()/2);
      tVoxel.y() = y+1;
      stepY = -1;
    }

    if (raydir.z() >= 0)
    {
      tVoxel.z() = z/dimensions_.z();
      stepZ = 1;
    }
    else {
     // tVoxel.z() = start.z() + (voxel_.z()/2);
     // tVoxel.z() = (z-1)/dimensions_.z();
      tVoxel.z() = z-1;
      stepZ = -1;
    }

    std::cout << "step: " << stepX << " " << stepY << " " << stepZ << std::endl;
    std::cout << "raydir: " << raydir.x() << " " << raydir.y() << " " << raydir.z() << std::endl;

    std::cout << "tVoxel: " << tVoxel.x() << " " << tVoxel.y() << " " << tVoxel.z() << std::endl;

  //  Eigen::Vector3i tt(tVoxel.x(),tVoxel.y(),tVoxel.z());

 //   tVoxel = voxelToCartesian(tt);


    Eigen::Vector3f coord3;
     x_length = (tVoxel.x() - centroid_voxel_.x()) * voxel_.x();
     y_length = (tVoxel.y() - centroid_voxel_.y()) * voxel_.y();
     z_length = (tVoxel.z() - centroid_voxel_.z()) * voxel_.z();

    coord3.x() = x_length + centroid_.x();
    coord3.y() = y_length + centroid_.y();
    coord3.z() = z_length + centroid_.z();





 //   float voxelMaxX  = vmin.x() + tVoxel.x()*bbx_.x();
 //   float voxelMaxY  = vmin.y() + tVoxel.y()*bbx_.y();
 //   float voxelMaxZ  = vmin.z() + tVoxel.z()*bbx_.z();

    float voxelMaxX  = coord3.x();
    float voxelMaxY  = coord3.y();
    float voxelMaxZ  = coord3.z();

    publish_topics_->robotPositionsMsg(coord3);


    Eigen::Vector3f tMax;
    tMax.x() = tmin + (voxelMaxX-start.x()) / raydir.x();
    tMax.y() = tmin + (voxelMaxY-start.y()) / raydir.y();
    tMax.z() = tmin + (voxelMaxZ-start.z()) / raydir.z();

    Eigen::Vector3f tDelta;
    tDelta.x() = voxel_.x()/fabs(raydir.x());
    tDelta.y() = voxel_.y()/fabs(raydir.y());
    tDelta.z() = voxel_.z()/fabs(raydir.z());



    std::cout << "voxel_: " << voxel_.x() << " " << voxel_.y() << " " << voxel_.z() << std::endl;
    std::cout << "tDelta: " << tDelta.x() << " " << tDelta.y() << " " << tDelta.z() << std::endl;

    // REPLACE
    // WITH SMITHs ALGORITHM
    // An Efficient and Robust Ray–Box Intersection Algorithm



   // Eigen::Vector3f tMax;
    /*
    if (raydir.x() < 0.0) tMax.x() = (vmin.x() - P0.x()) / raydir.x();
    if (raydir.x() > 0.0) tMax.x() = (vmax.x() - P0.x()) / raydir.x();
    if (raydir.y() < 0.0) tMax.y() = (vmin.y() - P0.y()) / raydir.y();
    if (raydir.y() > 0.0) tMax.y() = (vmax.y() - P0.y()) / raydir.y();
    if (raydir.z() < 0.0) tMax.z() = (vmin.z() - P0.z()) / raydir.z();
    if (raydir.z() > 0.0) tMax.z() = (vmax.z() - P0.z()) / raydir.z();
*/
    // REPLACE

/*
    int stepX = 1.0; int stepY = 1.0; int stepZ = 1.0;
    int outX = dimensions_.x(); int outY = dimensions_.y(); int outZ = dimensions_.z();
    if (raydir.x() < 0.0) {stepX = -1; outX = -1;}
    if (raydir.y() < 0.0) {stepY = -1; outY = -1;}
    if (raydir.z() < 0.0) {stepZ = -1; outZ = -1;}

    const Eigen::Vector3i v = cartesianToVoxel(tMax);
    int x=v.x();
    int y=v.y();
    int z=v.z();
*/


    ROS_INFO("TEST");
    std::cout << "tMax: " << tMax.x() << " " << tMax.y() << " " << tMax.z() << std::endl;
   // publish_topics_->robotPositionsMsg(tMax);

    std::cout << x << " " << y << " " << z << std::endl;


    while ( (x<dimensions_.x()) && (x>=0) &&
            (y<dimensions_.y()) && (y>=0) &&
            (z<dimensions_.z()) && (z>=0) )
    {
      std::cout << x << " " << y << " " << z << std::endl;
     // Eigen::Vector3i b(x,y,z);
    //  Eigen::Vector3f vcoord = voxelToCartesian(b);
    //  publish_topics_->robotPositionsMsg(vcoord);
      vgrid[z][y][x].value = UNKNOWN;
   //   ray.push_back(b);



      if (tMax.x() < tMax.y())
      {
        if (tMax.x() < tMax.z())
        {
          x = x + stepX;
          tMax.x() = tMax.x() + tDelta.x();
        }else{
          z = z + stepZ;
          tMax.z() = tMax.z() + tDelta.z();
        }
      }else{
        if (tMax.y() < tMax.z())
        {
          y = y + stepY;
          tMax.y() = tMax.y() + tDelta.y();
        }else{
          z = z + stepZ;
          tMax.z() = tMax.z() + tDelta.z();
        }
      }
    }

  }







}


// Smits’ method
double VoxelGrid::intersect(const Eigen::Vector3f P0, const Eigen::Vector3f P1) const
{
  Eigen::Vector3f vmin;
  Eigen::Vector3f vmax;
  vmin.x() = centroid_.x() - (bbx_.x()/2);
  vmin.y() = centroid_.y() - (bbx_.y()/2);
  vmin.z() = centroid_.z() - (bbx_.z()/2);
  vmax.x() = centroid_.x() + (bbx_.x()/2);
  vmax.y() = centroid_.y() + (bbx_.y()/2);
  vmax.z() = centroid_.z() + (bbx_.z()/2);

  const Eigen::Vector3f raydir(P1 - P0);

  float tmin, tmax, tymin, tymax, tzmin, tzmax;
  if (raydir.x() >= 0) {
    tmin = (vmin.x() - P0.x()) / raydir.x();
    tmax = (vmax.x() - P0.x()) / raydir.x();
  }
  else {
    tmin = (vmax.x() - P0.x()) / raydir.x();
    tmax = (vmin.x() - P0.x()) / raydir.x();
  }

  if (raydir.y() >= 0) {
    tymin = (vmin.y() - P0.y()) / raydir.y();
    tymax = (vmax.y() - P0.y()) / raydir.y();
  }
  else {
    tymin = (vmax.y() - P0.y()) / raydir.y();
    tymax = (vmin.y() - P0.y()) / raydir.y();
  }

  if ( (tmin > tymax) || (tymin > tmax) )
    //return false;
    ROS_INFO("Intersect - ERROR");

  if (tymin > tmin)
    tmin = tymin;
  if (tymax < tmax)
    tmax = tymax;

  if (raydir.z() >= 0) {
    tzmin = (vmin.z() - P0.z()) / raydir.z();
    tzmax = (vmax.z() - P0.z()) / raydir.z();
  }
  else {
    tzmin = (vmax.z() - P0.z()) / raydir.z();
    tzmax = (vmin.z() - P0.z()) / raydir.z();
  }

  if( (tmin > tzmax) || (tzmin > tmax) )
    //return false;
    ROS_INFO("Intersect - ERROR");

  if (tzmin > tmin)
    tmin = tzmin;
  if (tzmax < tmax)
    tmax = tzmax;

  return tmin;
  //return true;
}








Eigen::Vector3f VoxelGrid::intersect3DLinePlane(const Eigen::Vector3f &P0, Eigen::Vector3f &P1,
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


void VoxelGrid::analyzeRay(std::vector<std::vector<Eigen::Vector3i > > rays, array_type &grid)
{

  //std::cerr << rays.size() - 1 << std::endl;

  bool traversObject = false;
  for(long int i=(long int)rays.size() - 1; i>(-1); i--)
//  for(int i=0; i<rays.size(); i++)
  {

    for(long int ii=(long int)rays[i].size() - 1; ii>(-1); ii--)
  //  for( int ii=0; ii<rays[i].size(); ii++)
    {
      int zz = rays[i][ii].z();
      int yy = rays[i][ii].y();
      int xx = rays[i][ii].x();


     // if(i==100)
     // {
     //   std::cerr << zz<<yy<<xx << std::endl;
     // }



      if( grid[zz][yy][xx].value == OCCUPIED){
        traversObject = true;
        break;
      }



      //else if( grid[zz][yy][xx] == UNKNOWN && !traversObject){

       // grid[zz][yy][xx] = FREE;

      //}
    }

    int zz = rays[i][0].z();
    int yy = rays[i][0].y();
    int xx = rays[i][0].x();

    if(!traversObject)
      grid[zz][yy][xx].value = FREE;

    traversObject = false;
  }
}

double VoxelGrid::computeEntropy(array_type &grid)
{
  double entropy = 0.0;

  std::vector<Eigen::Vector3i> uvox = getUnknownVoxels(grid);

  for(unsigned int i=0; i<uvox.size(); i++)
  {
    int x,y,z;
    x = uvox[i].x();
    y = uvox[i].y();
    z = uvox[i].z();

    double p = grid[z][y][x].observation_prob;

    entropy += p*log(p);
  }

  entropy = entropy * (-1);
  return entropy;
}

double VoxelGrid::computeExpectation(array_type &grid)
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

double VoxelGrid::computeExpectation(array_type &grid, std::vector<Eigen::Vector3i> &uvox)
{
  double expectaion = 0.0;

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


double VoxelGrid::computeFarthestPosistion(std::vector<Eigen::Vector3f>& sampling_positions,
                                     const Eigen::Vector3f& P)
{
  float dist = 0.0;
  int index = 0;

  for(unsigned int ii=0; ii<sampling_positions.size(); ii++)
  {
    float tmp_dist = sqrt(pow(P.x() - sampling_positions[ii].x(),2) +
                          pow(P.y() - sampling_positions[ii].y(),2) +
                          pow(P.z() - sampling_positions[ii].z(),2));
    if(tmp_dist>dist){
      dist = tmp_dist;
      index=ii;
    }
  }
  return dist;
}



std::vector<Eigen::Vector3i> VoxelGrid::getUnknownVoxels(array_type &grid)
{
  std::vector<Eigen::Vector3i> uvox;

  for (int zz = dimensions_.z()-1; zz >(-1) ; zz--) {
    for (int yy = 0; yy < dimensions_.y(); ++yy){
      for(int xx = 0; xx < dimensions_.x(); ++xx){
        if( grid[zz][yy][xx].value == UNKNOWN )
        {
          Eigen::Vector3i vox(xx, yy, zz);
          uvox.push_back(vox);
        }
      }
    }
  }
  return uvox;
}

std::vector<Eigen::Vector3i> VoxelGrid::getVisibleVoxels(array_type &grid)
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


std::vector<Eigen::Vector3i> VoxelGrid::getUnknownVoxelsFOV(const Eigen::Vector3f P0, array_type &grid, float direction)
{
  // field of view angle
  float fov = 7.5;
  Eigen::Vector2f p1(centroid_.x(), centroid_.y());
  Eigen::Vector2f p2(P0.x(), P0.y());

  float theta = direction * M_PI/180;
  Eigen::Matrix2f R;
  R << cos(theta), -sin(theta),
       sin(theta),  cos(theta);

  Eigen::Vector2f p2f(0.0,0.0);
  p2f = R*(p1-p2) + p2;
  p1 = p2f;

  Eigen::Vector2f normal = computeNormal(p1,p2,fov);

  std::vector<Eigen::Vector3i> uvox_tmp;
  Eigen::Vector3f V0;
  for (int zz = dimensions_.z()-1; zz >(-1) ; zz--) {
    for (int yy = 0; yy < dimensions_.y(); ++yy){
      for(int xx = 0; xx < dimensions_.x(); ++xx){
        if( grid[zz][yy][xx].value == UNKNOWN )
        {
          Eigen::Vector3i vox(xx, yy, zz);
          V0 = voxelToCartesian(vox);

          Eigen::Vector2f p2d(V0.x(), V0.y());

          Eigen::Vector2f plo = p2d-p2;
          float angle = normal.dot(plo);

          if(angle < 0.0)
          {
            uvox_tmp.push_back(vox);
          }
        }
      }
    }
  }


  normal = computeNormal(p1,p2,-fov);

  std::vector<Eigen::Vector3i> uvox_tmp2;
  for( int vox=(int)uvox_tmp.size()-1; vox>(-1); vox--)
  {
    const Eigen::Vector3i v(uvox_tmp[vox].x(),uvox_tmp[vox].y(),uvox_tmp[vox].z());
    if( grid[v.z()][v.y()][v.x()].value == UNKNOWN )
    {
      //Eigen::Vector3i vox(xx, yy, zz);
      V0 = voxelToCartesian(v);

      Eigen::Vector2f p2d(V0.x(), V0.y());

      Eigen::Vector2f plo = p2d-p2;
      float angle = normal.dot(plo);

      if(angle > 0.0)
      {
        uvox_tmp2.push_back(v);
      }
    }
  }

  return uvox_tmp2;
}

Eigen::Vector2f VoxelGrid::computeNormal(Eigen::Vector2f p1, Eigen::Vector2f p2, float fov)
{
  float theta = fov * M_PI/180;
  Eigen::Matrix2f R;
  R << cos(theta), -sin(theta),
       sin(theta),  cos(theta);

  Eigen::Vector2f p2f(0.0,0.0);
  p2f = R*(p1-p2) + p2;
  Eigen::Vector2f p(p2f.x(), p2f.y());

  Eigen::Vector2f pp = p-p2;
  Eigen::Vector2f normal = pp.unitOrthogonal();

  return normal;
}





const Eigen::Vector3f VoxelGrid::findNBV()
{

  std::vector<Eigen::Vector3i> uvoxO = getUnknownVoxels(voxel_grid_);
  array_type vgrid(boost::extents[dimensions_.z()][dimensions_.y()][dimensions_.x()]);

  std::vector<Eigen::Vector3i> uvox;
  int minvox = (int)uvoxO.size();
  int pos = 0;
  for(unsigned int ii=0; ii< (unsigned int)sampling_positions_.size(); ii++)
  {
    vgrid = voxel_grid_;

    const Eigen::Vector3f P(sampling_positions_[ii].x(), sampling_positions_[ii].y(), sampling_positions_[ii].z());

    //this->rayTraversal(P, uvoxO, vgrid);

    rayTraversal2(P, uvoxO, vgrid, false);
    uvox = getUnknownVoxels(vgrid);

    std::cerr << "unknown: " << (int)uvox.size() << std::endl;

    int nvox = (int)uvox.size();
    if( nvox < minvox ){
      pos = ii;
      minvox = nvox;
      virtual_grid_ = vgrid;
    }
  }

  std::cerr << "old unknown: " << (int)uvoxO.size() << std::endl;
  std::cerr << "new unknown: " << minvox << std::endl;

  const Eigen::Vector3f new_san_pos = sampling_positions_[pos];
  // erase the new scan position from the sampling position vector
  sampling_positions_.erase(sampling_positions_.begin() + pos);

  return new_san_pos;
}

void VoxelGrid::estimateVoxelGrid(array_type& vgrid,
                                  const Eigen::Vector3f& P,
                                  std::vector<Eigen::Vector3i>& uvox)
{
  //array_type vgrid(boost::extents[dimensions_.z()][dimensions_.y()][dimensions_.x()]);
  //vgrid = voxel_grid;

  rayTraversal2(P, uvox, vgrid, false);

  /*
  std::vector<Eigen::Vector3i> uvox;
  int minvox = (int)uvoxO.size();
  int pos = 0;




    this->

    uvox = getUnknownVoxels(vgrid);

    std::cerr << "unknown: " << (int)uvox.size() << std::endl;

    int nvox = (int)uvox.size();
    if( nvox < minvox){
      pos = ii;
      minvox = nvox;
      virtual_grid_ = vgrid;
    }


  std::cerr << "old unknown: " << (int)uvoxO.size() << std::endl;
  std::cerr << "new unknown: " << minvox << std::endl;

  const Eigen::Vector3f new_san_pos = sampling_positions_[pos];
  // erase the new scan position from the sampling position vector
  sampling_positions_.erase(sampling_positions_.begin() + pos);
*/
 // return vgrid;
}

void VoxelGrid::getOccupiedVoxels()
{
  for (int zz = 0; zz < dimensions_.z(); ++zz) {
    for (int yy = 0; yy < dimensions_.y(); ++yy){
      for(int xx = 0; xx < dimensions_.x(); ++xx){
        if( voxel_grid_[zz][yy][xx].value == OCCUPIED )
        {
          Eigen::Vector3i v(xx,yy,zz);
          occupied_voxels_.push_back(v);
        }
      }
    }
  }
}

void VoxelGrid::computeUnknownObjectProb(std::vector<Eigen::Vector3i> &uvox)
{
  int vox_threshold = 10;
  float vox_size = voxel_.x();


  getOccupiedVoxels();

  for( int vox=(int)uvox.size()-1; vox>(-1); vox--)
  {
    Eigen::Vector3i uvox_coord(uvox[vox]);
    for( int occ=0; occ<(int)occupied_voxels_.size(); occ++ )
    {
      Eigen::Vector3i ovox_coord(occupied_voxels_[occ]);

      Eigen::Vector3f a = voxelToCartesian(uvox_coord);
      Eigen::Vector3f b = voxelToCartesian(ovox_coord);

      float dist = euclidean_distance(a,b);






    }

  }


}

void VoxelGrid::setObservationProb(std::vector<Eigen::Vector3i> &uvox, float value)
{
  for( int vox=(int)uvox.size()-1; vox>(-1); vox--)
  {
    Eigen::Vector3i vcoord(uvox[vox]);
    voxel_grid_[vcoord.z()][vcoord.y()][vcoord.x()].observation_prob = value;
    //voxel_grid_[vcoord.z()][vcoord.y()][vcoord.x()].enclosed_prob = 1.0;
  }


}


void VoxelGrid::voxelGridToMsg()
{
  //publish_topics_->voxelGridToMsg(voxel_grid_, dimensions_);
}


void VoxelGrid::voxelGridMsg()
{


  visualization_msgs::Marker voxel_grid_Occupied_msg;
  visualization_msgs::Marker voxel_grid_free_msg;
  visualization_msgs::Marker voxel_grid_unknown_msg;
  visualization_msgs::Marker virtual_grid_unknown_msg_;
  visualization_msgs::Marker entropy_msg_;

  voxel_grid_Occupied_msg.header.frame_id = "/map";
  voxel_grid_Occupied_msg.header.stamp = ros::Time::now();
  voxel_grid_Occupied_msg.ns = "Occupied";
  voxel_grid_Occupied_msg.id = 0;
  voxel_grid_Occupied_msg.type = visualization_msgs::Marker::POINTS;
  voxel_grid_Occupied_msg.action = visualization_msgs::Marker::ADD;

  voxel_grid_free_msg.header.frame_id = "/map";
  voxel_grid_free_msg.header.stamp = ros::Time::now();
  voxel_grid_free_msg.ns = "Free";
  voxel_grid_free_msg.id = 0;
  voxel_grid_free_msg.type = visualization_msgs::Marker::POINTS;
  voxel_grid_free_msg.action = visualization_msgs::Marker::ADD;

  voxel_grid_unknown_msg.header.frame_id = "/map";
  voxel_grid_unknown_msg.header.stamp = ros::Time::now();
  voxel_grid_unknown_msg.ns = "Unknown";
  voxel_grid_unknown_msg.id = 0;
  voxel_grid_unknown_msg.type = visualization_msgs::Marker::POINTS;
  voxel_grid_unknown_msg.action = visualization_msgs::Marker::ADD;

  virtual_grid_unknown_msg_.header.frame_id = "/map";
  virtual_grid_unknown_msg_.header.stamp = ros::Time::now();
  virtual_grid_unknown_msg_.ns = "Vi_Unknown";
  virtual_grid_unknown_msg_.id = 0;
  virtual_grid_unknown_msg_.type = visualization_msgs::Marker::POINTS;
  virtual_grid_unknown_msg_.action = visualization_msgs::Marker::ADD;

  entropy_msg_.header.frame_id = "/map";
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

        if( voxel_grid_[zz][yy][xx].value == OCCUPIED  )
        {
          voxel_grid_Occupied_msg.points.push_back(p);
        }

        if( voxel_grid_[zz][yy][xx].value == FREE )
        {
          voxel_grid_free_msg.points.push_back(p);
        }

        if( voxel_grid_[zz][yy][xx].value == UNKNOWN )
        {
          voxel_grid_unknown_msg.points.push_back(p);
        }

        if( virtual_grid_[zz][yy][xx].value == UNKNOWN )
        {
          virtual_grid_unknown_msg_.points.push_back(p);
        }

        if( voxel_grid_[zz][yy][xx].observation_prob > 0.0 )
        {
          std_msgs::ColorRGBA color;
          color.r = 1.0;
          color.g =  voxel_grid_[zz][yy][xx].observation_prob;
          color.b = 0.0;
          color.a = 1.0;
          entropy_msg_.colors.push_back(color);
          entropy_msg_.points.push_back(p);
        }

      }
    }
  }

  voxel_grid_Occupied_msg.scale.x = 0.005;
  voxel_grid_Occupied_msg.scale.y = 0.005;
  voxel_grid_free_msg.scale.x = 0.005;
  voxel_grid_free_msg.scale.y = 0.005;
  voxel_grid_unknown_msg.scale.x = 0.005;
  voxel_grid_unknown_msg.scale.y = 0.005;
  virtual_grid_unknown_msg_.scale.x = 0.005;
  virtual_grid_unknown_msg_.scale.y = 0.005;
  entropy_msg_.scale.x = 0.005;
  entropy_msg_.scale.y = 0.005;


  voxel_grid_Occupied_msg.color.r = 0.0;
  voxel_grid_Occupied_msg.color.g = 0.0;
  voxel_grid_Occupied_msg.color.b = 1.0;
  voxel_grid_Occupied_msg.color.a = 1.0;

  voxel_grid_free_msg.color.r = 0.0;
  voxel_grid_free_msg.color.g = 1.0;
  voxel_grid_free_msg.color.b = 0.0;
  voxel_grid_free_msg.color.a = 1.0;

  voxel_grid_unknown_msg.color.r = 1.0;
  voxel_grid_unknown_msg.color.g = 1.0;
  voxel_grid_unknown_msg.color.b = 0.0;
  voxel_grid_unknown_msg.color.a = 1.0;

  virtual_grid_unknown_msg_.color.r = 1.0;
  virtual_grid_unknown_msg_.color.g = 0.0;
  virtual_grid_unknown_msg_.color.b = 0.0;
  virtual_grid_unknown_msg_.color.a = 1.0;

 // entropy_msg_.color.r = 0.0;
 // entropy_msg_.color.g = 0.0;
 // entropy_msg_.color.b = 0.0;
  entropy_msg_.color.a = 1.0;

  publish_topics_->newVisualizationMarker(voxel_grid_Occupied_msg);
  publish_topics_->newVisualizationMarker(voxel_grid_free_msg);
  publish_topics_->newVisualizationMarker(voxel_grid_unknown_msg);
  publish_topics_->newVisualizationMarker(virtual_grid_unknown_msg_);
  publish_topics_->newVisualizationMarker(entropy_msg_);


}

void VoxelGrid::mrfVisul()
{
  visualization_msgs::Marker mrf_msg;

  mrf_msg.header.frame_id = "/map";
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


          if( mrfs_[i][zz][yy][xx].value == OCCUPIED )
            mrf_msg.points.push_back(p);

        }
      }
    }


  }



  mrf_msg.scale.x = 0.005;
  mrf_msg.scale.y = 0.005;

  mrf_msg.color.r = 0.0;
  mrf_msg.color.g = 1.0;
  mrf_msg.color.b = 1.0;
  mrf_msg.color.a = 1.0;

  publish_topics_->newVisualizationMarker(mrf_msg);

}





