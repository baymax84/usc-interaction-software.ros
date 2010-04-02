#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxtypes.h>
#include <image_transport/image_transport.h>


image_transport::Subscriber sub_;
image_transport::Publisher pub_;
sensor_msgs::ImageConstPtr last_msg_;
sensor_msgs::CvBridge img_bridge_;

ros::Publisher info_pub_;
ros::Subscriber info_sub_;

int border;

void info_cb(const sensor_msgs::CameraInfoConstPtr& msg)
{
	sensor_msgs::CameraInfo newmsg = *msg;
	newmsg.height += 2*border;
	newmsg.width += 2*border;

	newmsg.P[2] += border;
	newmsg.P[5] += border;

	info_pub_.publish(newmsg);
}

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  if (msg->encoding.find("bayer") != std::string::npos)
    boost::const_pointer_cast<sensor_msgs::Image>(msg)->encoding = "mono8";

  if (img_bridge_.fromImage(*msg, "bgr8"))
  {
    IplImage *orig, *dest;
    orig = img_bridge_.toIpl();
		CvPoint offset; offset.x = border; offset.y = border;
    // *dest = cvCreateMat(orig->height + border*2, orig->width + border*2, orig->depth);
		dest = cvCreateImage(cvSize(orig->width+border*2,orig->height+border*2), 8, 3 );
		cvZero(dest);
    //cvCopyMakeBorder(orig,dest,border,border,border,border,IPL_BORDER_CONSTANT);
    cvCopyMakeBorder(orig,dest,offset,IPL_BORDER_CONSTANT,cvScalar(0,0,0));

		//cvShowImage("display",dest);
		//cvWaitKey(10);
		sensor_msgs::ImagePtr newmsg = sensor_msgs::CvBridge::cvToImgMsg(dest, "bgr8");
		pub_.publish( newmsg );
  }
  else
    ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
}

int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "image_letterboxer" );
  ros::NodeHandle nh;
  nh.param("border", border, 100 );
	//cvNamedWindow("display",1);
  image_transport::ImageTransport it(nh);
  sub_ = it.subscribe("image_raw", 1, &image_cb);
  pub_ = it.advertise("letterbox/image_raw", 1);
	info_sub_ = nh.subscribe( "camera_info", 1, &info_cb);
	info_pub_ = nh.advertise<sensor_msgs::CameraInfo>( "letterbox/camera_info", 1 );
  ros::spin();

  return 0;
}
