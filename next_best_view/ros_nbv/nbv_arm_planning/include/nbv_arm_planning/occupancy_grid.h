/*
 * occupancy_grid.h
 *
 *  Created on: Apr 2, 2011
 *      Author: potthast
 */

#ifndef OCCUPANCY_GRID_H_
#define OCCUPANCY_GRID_H_

#include <algorithm>
#include "boost/multi_array.hpp"
#include <time.h>

#include <nbv_arm_planning/publish_topics.h>
#include <nbv_arm_planning/voxel_grid.h>

class OccupancyGrid
{

  typedef struct voxel {
    float p;
    bool update;
    int state;
    float observation_prob;
    float enclosed_prob;
  } voxel_t;

  typedef pcl::PointXYZRGB PointRGB;



public:

  typedef boost::multi_array<voxel_t, 3> array_type;

  OccupancyGrid(Eigen::Vector3f box, Eigen::Vector3i dim,
                Eigen::Vector4f centroid, std::vector<Eigen::Vector3f> table_top_bbx,
                PublishTopics& publish_topics);

  // update the occupancy grid using Baye's Theory
  void measurementsUpdate();
  float bayesianUpdate();

  void createMRFs();
  void evalMrf();
  void ICM(array_type &mrf, array_type &hl, Eigen::Vector3i &dim);

  std::vector<Eigen::Vector3i> getUnknownVoxels(array_type &grid);
  std::vector<Eigen::Vector3i> getUnknownVoxelsFOV(const Eigen::Vector3f P0, array_type &grid, float direction);
  Eigen::Vector2f computeNormal(Eigen::Vector2f p1, Eigen::Vector2f p2, float fov);
  void setStates();
  void voxelGridMsg();
  void mrfVisul();
  void MRFToVG();

  float eulcideanDistance(const Eigen::Vector3f p1, const Eigen::Vector3f p2);

  void rayTraversal( const Eigen::Vector3f P0, std::vector<Eigen::Vector3i> &uvox, array_type &vgrid,bool real);
  Eigen::Vector3f intersect3DLinePlane(const Eigen::Vector3f &P0, Eigen::Vector3f &P1,
                                             Eigen::Vector3f &V0, const Eigen::Vector3f &n);
  double computeExpectation(array_type &grid);
  std::vector<Eigen::Vector3i> getVisibleVoxels(array_type &grid);

  // region growing in grid
  std::vector<Eigen::Vector3i> growRegion(Eigen::Vector3i vox, int v_size);
  // find volumes in the voxel grid
  std::vector<std::vector<Eigen::Vector3i> > findVolumes(int v_size);

  void wall();
  void colorUnknownVoxels(std::vector<Eigen::Vector3i> uvox);

  // set and get functions
  void setRotAngle(float rot){rot_angle_ = rot;};
  void setMeasurements(pcl::PointCloud<PointRGB>::ConstPtr &measurement){measurement_=measurement;};
  float getRotAngle(){return rot_angle_;};
  void setGrid(array_type& grid){occupancy_grid_ = grid;};
  array_type getGrid(){return occupancy_grid_;};
  void setObjectDimensions(std::vector<Eigen::VectorXf>& objd){objd_ = objd;};
  void setViewDir(Eigen::Vector3f view_dir){view_dir_=view_dir;};

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


  inline float multivariateGaussianDistribution(const Eigen::Vector3f& x,
                                                const Eigen::VectorXf& mu,
                                                const Eigen::MatrixXf& sigma,
                                                const Eigen::MatrixXf& sigmaInv)
  {
    float k = sigma.diagonalSize();
    float f1 =  1.0 / (pow((2*M_PI),(k/2.0)) * pow((sigma.determinant()),(1.0/2.0)));
    float f2 = (x-mu).transpose()*sigmaInv*(x-mu);
    float p = f1*exp(-1.0/2.0*f2 );

    return p;
  }

  inline
    float bayesianUpdate(float &p_z_given_r, float &p_r_given_z)
    {
      float p = 0.0;
      p = (p_r_given_z * p_z_given_r) / ( (1-p_r_given_z)*(1-p_z_given_r) + (p_r_given_z * p_z_given_r));
      return p;
    }

  inline float prob_kernel(float uv)
  {
    float f = pow(0.98,uv);
    return f;

  }

  inline float compute_expectation(float uv)
  {
    float f = prob_kernel(uv);
    float expectation = 0 * (1.0-f) + 1 * f;
    return expectation;
  }



  // dimensions of the voxel grid
  Eigen::Vector3i dimensions_;


private:
  // occupancy grid
  array_type occupancy_grid_;

  // Markov Random Field
  std::vector<array_type> mrfs_;
  std::vector<Eigen::VectorXf> objd_;

  PublishTopics *publish_topics_;

  Eigen::Vector3f bbx_;
  Eigen::Vector4f centroid_;
  Eigen::Vector3f voxel_;
  Eigen::Vector3f centroid_voxel_;

  Eigen::Vector3f view_dir_;

  // occupancy grid rotation angle
  float rot_angle_;

  // new measurements
  pcl::PointCloud<PointRGB>::ConstPtr measurement_;
  pcl::PointCloud<PointRGB> cloud_;

  std::vector<Eigen::Vector3i> ray_;

};
#endif /* OCCUPANCY_GRID_H_ */
