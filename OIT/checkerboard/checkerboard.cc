/*
 *  Checkerboard detector node for ROS
 *  Copyright (C) 2010
 *     David Feil-Seifer
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/CvBridge.h>
#include <tf/transform_broadcaster.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>

sensor_msgs::CvBridge img_bridge;
tf::TransformBroadcaster* tb;

std::string checkerboard_frame;

#define RTOD(a) ((a) * 180.0 / M_PI )

int nx = 7;
int ny = 6;

std::vector<cv::Point3f> getObjPoints( int x, int y, double size )
{
  bool xodd = x % 2 == 1; bool yodd = x % 2 == 1;
  std::vector<cv::Point3f> ret;

  for( int i = 0; i < y; i++ )
  {
    double ycoord = size*(-y/2+i);
    if( !yodd ) ycoord += size/2.0;

    for( int j = 0; j < x; j++ )
    {
      double xcoord = size*(-x/2+j);
      if( !xodd ) xcoord += size/2.0;
      cv::Point3f coord = cv::Point3f( xcoord, ycoord, 0);
      ret.push_back(coord);
    }
  }
  return ret;
}


void image_cb( const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& infomsg )
{
  cv::Mat img;
  try
  {
    img = img_bridge.imgMsgToCv(msg, "bgr8" );
    
    std::vector<cv::Point2f> corners;
    if( cv::findChessboardCorners( img, cvSize(nx,ny), corners ) )
    {
      cv::drawChessboardCorners(img, cvSize(nx,ny), corners, true );

      std::vector<cv::Point2f> imgpts;
      double fR3[3], fT3[3];
      cv::Mat rvec(3, 1, CV_64FC1, fR3);
      cv::Mat tvec(3, 1, CV_64FC1, fT3);

      image_geometry::PinholeCameraModel pcam;
      pcam.fromCameraInfo(infomsg);
      std::vector<cv::Point3f> objpts = getObjPoints(nx,ny,.108);

      cv::solvePnP( objpts, corners, pcam.intrinsicMatrix(), pcam.distortionCoeffs(), rvec, tvec );
      
      tf::Transform checktf;
      checktf.setOrigin( tf::Vector3(fT3[0], fT3[1], fT3[2] ) );
      double rx = fR3[0], ry = fR3[1], rz = fR3[2];
			tf::Quaternion quat;
      quat.setRPY(rx, ry, rz);
			checktf.setRotation( quat );
      tb->sendTransform(tf::StampedTransform(checktf, msg->header.stamp, msg->header.frame_id, checkerboard_frame ));
    }
 }
  catch( sensor_msgs::CvBridgeException cvbe )
  {
  	ROS_INFO( "failed" );
    ROS_WARN( "cv_bridge could not get Mat from Image Msg: %s", cvbe.what() );
  }

  // bundle up image with checkerboard dots drawn for export
  cv::imshow("corners", img );
  cv::waitKey(10);
 }

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "checkerboard" );
  ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
  tb = new tf::TransformBroadcaster();
	cvNamedWindow( "corners", 1 );
	nh_priv.param("child_frame_id", checkerboard_frame,std::string("checkerboard"));
  image_transport::ImageTransport it(nh);
  image_transport::CameraSubscriber image_sub = it.subscribeCamera("image", 1, image_cb );
	ROS_INFO("getting ready to spin");
  ros::spin();
  return 0;

}
