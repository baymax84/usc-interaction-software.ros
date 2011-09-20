/*
 * stitch_clouds.h
 *
 *  Created on: Aug 15, 2011
 *      Author: potthast
 */

#ifndef STITCH_CLOUDS_H_
#define STITCH_CLOUDS_H_

#include <pcl/io/pcd_io.h>
#include "pcl/point_types.h"
#include <pcl/filters/voxel_grid.h>
#include <boost/lexical_cast.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class StitchClouds
{
public:

  typedef struct command_arg {
    bool downsampling;
    float leaf_size;
    std::string o_name;
    std::vector<std::string> i_name;
  } command_arg_t;


  StitchClouds(command_arg_t &command_arg);

private:
};


#endif /* STITCH_CLOUDS_H_ */
