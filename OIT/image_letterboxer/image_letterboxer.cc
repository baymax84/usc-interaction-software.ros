#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/cxtypes.h>
#include <image_transport/image_transport.h>


image_transport::Subscriber sub_;
image_transport::Publisher pub_;
sensor_msgs::ImageConstPtr last_msg_;
sensor_msgs::CvBridge img_bridge_;

int border;

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  if (msg->encoding.find("bayer") != std::string::npos)
    boost::const_pointer_cast<sensor_msgs::Image>(msg)->encoding = "mono8";

  if (img_bridge_.fromImage(*msg, "bgr8"))
  {
    IplImage *orig;
    orig = img_bridge_.toIpl();
    cv::Mat dest(orig->height + border*2, orig->width + border*2, orig->depth);
    cv::copyMakeBorder(orig,dest,border,border,border,border,BORDER_CONSTANT);
  }
  else
    ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
}

int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "image_letterboxer" );
  ros::NodeHandle nh("~");
  nh.param("border", border, 100 );

  image_transport::ImageTransport it(nh);
  sub_ = it.subscribe("image_raw", 1, &image_cb);
  pub_ = it.advertise("letterbox/image_raw", 1); 
  ros::spin();

  return 0;
}
