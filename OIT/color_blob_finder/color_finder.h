#ifndef _COLOR_FINDER_H_
#define _COLOR_FINDER_H_

#include "ros/ros.h"
#include "opencv/cv.h"
#include "oit_msgs/BlobArray.h"

class ColorFinder
{
  ros::Publisher blobs_pub;
  ros::Publisher world_pub;
  IplImage *backproject_img_, *mask_, *hue_;

  oit_msgs::BlobArray blobs_;

  // color histogram settings
  std::string color_histfile_;
  int smin_, vmin_, vmax_, hdims_;
  int min_area_;
  CvHistogram *hist_;
  CvMemStorage* storage_;
  IplConvKernel *kernel_;

  public:
    void init( std::string, std::string, int min_area = 5 );
    void image_cb( IplImage* img );
    void find_blobs(ros::Time t);

    std::vector<oit_msgs::Blob> get_blobs() { return blobs_.blobs; }
    
};

#endif
