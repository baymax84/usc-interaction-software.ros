/*
 * voxel_grid.h
 *
 *  Created on: Dec 5, 2010
 *      Author: Christian Potthast
 */

#ifndef VOXEL_GRID_H_
#define VOXEL_GRID_H_

#include <algorithm>

//#include <Eigen/StdVector>
#include <nbv_main/publish_topics.h>
//#include <nbv_main/utilities.h>

#include "boost/multi_array.hpp"

//const float UNKNOWN = 0.5;
//const float OCCUPIED = 1.0;
//const float FREE = 0.0;

const int UNKNOWN = 2;
const int OCCUPIED = 1;
const int FREE = 0;
const float ENCLOSED_PROB = 0.5;

class VoxelGrid
{
  typedef pcl::PointXYZRGB PointRGB;
  //typedef boost::multi_array<float, 3> array_type;


public:

  typedef struct voxel {
    int value;
    float observation_prob;
    float enclosed_prob;
  } voxel_t;


  typedef boost::multi_array<voxel_t, 3> array_type;

  VoxelGrid(Eigen::Vector3f box, Eigen::Vector3i dim,
            Eigen::Vector4f centroid, std::vector<Eigen::Vector3f> table_top_bbx,
            PublishTopics& publish_topics);
  void setGridPoints(pcl::PointCloud<PointRGB>::ConstPtr &point);
  void setObjectDimensions(std::vector<Eigen::VectorXf>& objd){objd_ = objd;};
  void voxelGridError();
  void createMRFs();
  void evalMrf();
  void MRFToVG();
  void ICM(array_type &mrf, array_type &hl, Eigen::Vector3i &dim);
  //void rayTraversal( const Eigen::Vector3f P0);
  //void rayTraversal( const Eigen::Vector3f P0, std::vector<Eigen::Vector3i> &uvox, array_type &vgrid);
  void rayTraversal2( const Eigen::Vector3f P0, std::vector<Eigen::Vector3i> &uvox, array_type &vgrid,bool real);
  Eigen::Vector3f intersect3DLinePlane(const Eigen::Vector3f &P0, Eigen::Vector3f &P1,
                                             Eigen::Vector3f &V0, const Eigen::Vector3f &n);
  void analyzeRay(std::vector<std::vector<Eigen::Vector3i > > rays, array_type &grid);

  void traversal(const Eigen::Vector3f P0, std::vector<Eigen::Vector3i> &uvox, array_type &vgrid);
  double intersect(const Eigen::Vector3f P0, const Eigen::Vector3f P1) const;

  void voxelGridMsg();
  void mrfVisul();
  void setRotAngle(float rot){rot_angle_ = rot;};
  float getRotAngle(){return rot_angle_;};
  void voxelGridToMsg();

  double computeEntropy(array_type &grid);
  double computeExpectation(array_type &grid);
  double computeExpectation(array_type &grid, std::vector<Eigen::Vector3i> &uvox);
  double computeFarthestPosistion(std::vector<Eigen::Vector3f>& sampling_positions,
                                       const Eigen::Vector3f& P);

  std::vector<Eigen::Vector3i> getUnknownVoxels(array_type &grid);
  std::vector<Eigen::Vector3i> getVisibleVoxels(array_type &grid);
  std::vector<Eigen::Vector3i> getUnknownVoxelsFOV(const Eigen::Vector3f P0, array_type &grid, float direction);
  Eigen::Vector2f computeNormal(Eigen::Vector2f p1, Eigen::Vector2f p2, float fov);

  const Eigen::Vector3f findNBV();
  void setSamplingPositions(std::vector<Eigen::Vector3f>& spos){sampling_positions_=spos;};
  array_type getVoxelGrid(){return voxel_grid_;};
  void setVoxelGrid(array_type& vgrid){voxel_grid_ = vgrid;};
  void setVirtuelGrid(array_type& vgrid){virtual_grid_ = vgrid;};

  void estimateVoxelGrid(array_type& vgrid,
                         const Eigen::Vector3f& P,
                         std::vector<Eigen::Vector3i>& uvox);

  void getOccupiedVoxels();
  void computeUnknownObjectProb(std::vector<Eigen::Vector3i> &uvox);
  void setObservationProb(std::vector<Eigen::Vector3i> &uvox, float value);

  inline Eigen::Vector3i cartesianToVoxel(const Eigen::Vector3f& point)
  {
    Eigen::Vector3i coord;
    Eigen::Vector3f p;
    p.x() = point.x()-centroid_.x();
    p.y() = point.y()-centroid_.y();
    p.z() = point.z()-centroid_.z();

    float theta = -rot_angle_;
    float xr = p.x()*cos(theta)-p.y()*sin(theta);
    float yr = p.x()*sin(theta)+p.y()*cos(theta);

    int vox_x = round( ( xr ) /voxel_.x());
    int vox_y = round( ( yr  ) /voxel_.y());
    int vox_z = round( ( point.z() - centroid_.z() ) /voxel_.z());

    coord.x() = vox_x + centroid_voxel_.x();
    coord.y() = vox_y + centroid_voxel_.y();
    coord.z() = vox_z + centroid_voxel_.z();

    return coord;
  }

  inline Eigen::Vector3f voxelToCartesian(const Eigen::Vector3i& vox)
  {
    Eigen::Vector3f coord;

    double x_length = (vox.x() - centroid_voxel_.x()) * voxel_.x();
    double y_length = (vox.y() - centroid_voxel_.y()) * voxel_.y();
    double z_length = (vox.z() - centroid_voxel_.z()) * voxel_.z();

    Eigen::Vector3f p;
    p.x() = x_length;
    p.y() = y_length;
   // p.z() = z_length-centroid_.z();

    float theta = rot_angle_;
    float xr = p.x()*cos(theta)-p.y()*sin(theta);
    float yr = p.x()*sin(theta)+p.y()*cos(theta);


    coord.x() = xr + centroid_.x();
    coord.y() = yr + centroid_.y();
    coord.z() = z_length + centroid_.z();

    return coord;
  }

  inline float euclidean_distance(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
  {
    float dist = sqrt(pow(a.x()-b.x(),2) + pow(a.y()-b.y(),2) + pow(a.z()-b.z(),2));
    return dist;
  }


  inline double kernel_f(double& u)
  {
    double x = 1/(sqrt(2*M_PI*0.3)) * exp(-pow(u-1,2) / (2*0.3) );
    return x;
  }

  inline float kernel3(float uv, float total)
  {
    float ratio = exp( (float)(-0.5*pow(total,2)) / (float)(pow(10,2)));
    return std::max(ratio,(float)0.5);
  }

  inline float gaussian_kernel(float uv)
  {
    float ratio = exp( (float)(-0.5*pow(uv,2)) / (float)(pow(3,2)));
    //return std::max(ratio,(float)0.5);
    return ratio;
  }

  inline float prob_kernel(float uv)
  {
    float f = pow(0.99,uv);
    return f;

  }

  inline float compute_expectation(float uv)
  {
    float f = prob_kernel(uv);
    float expectation = 0 * (1.0-f) + 1 * f;
    return expectation;
  }



  Eigen::Vector3i dimensions_;

private:
  // voxel grid
  array_type voxel_grid_;
  // voxel grid from 'virtual' scans
  array_type virtual_grid_;

  std::vector<Eigen::Vector3i> occupied_voxels_;

  std::vector<Eigen::Vector3f> sampling_positions_;

  std::vector<array_type> mrfs_;
  std::vector<Eigen::VectorXf> objd_;

  PublishTopics *publish_topics_;

  Eigen::Vector3f bbx_;
  Eigen::Vector4f centroid_;
  Eigen::Vector3f voxel_;
  Eigen::Vector3f centroid_voxel_;

  pcl::PointCloud<PointRGB> cloud_;

  float rot_angle_;

};


#endif /* VOXEL_GRID_H_ */
