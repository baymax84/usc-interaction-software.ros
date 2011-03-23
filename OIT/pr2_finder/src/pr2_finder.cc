/*
 * ir_finder
 * Copyright (c) 2010, David Feil-Seifer and Edward Kaszubski
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <oit_msgs/BlobArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "LinearMath/btMatrix3x3.h"
#include <math.h>

// need pinhole camera model
bool model_initialized = false;
double height = 0.0;//0.812;
double cam_height = 0.0;
double blob_dist_baseline;
double blob_dist_tolerance;
image_geometry::PinholeCameraModel pcam;
sensor_msgs::CvBridge img_bridge_;
tf::TransformBroadcaster* tb;
tf::TransformListener* listener;

IplImage* frame;
oit_msgs::BlobArrayConstPtr reds_, yellows_;

void setHeightFromPublishedTransform()
{
  tf::StampedTransform height_tf, cam_height_tf;
  try
  {
    listener->lookupTransform("/ovh_height", "/ovh", ros::Time(0), cam_height_tf);
    //listener->lookupTransform("/robot/ir", "/robot/base_link", ros::Time(0), height_tf);
  }
  catch (tf::TransformException ex)
  {
      ROS_ERROR("%s",ex.what());
      return;
  }
  cam_height = cam_height_tf.getOrigin().z();
	height = 1.25;
  //height = -height_tf.getOrigin().z();
}

void image_cb( const sensor_msgs::ImageConstPtr &img )
{
	sensor_msgs::Image img_msg = *(img.get());
	if( img_bridge_.fromImage( img_msg, "bgr8" ) )
	{
		frame = img_bridge_.toIpl();
	}

}


void camera_cb( const sensor_msgs::CameraInfoConstPtr& cam )
{
  //if( !model_initialized )
  {
    pcam.fromCameraInfo( cam );
    model_initialized = true;
  }
}

void red_cb(const oit_msgs::BlobArrayConstPtr& reds )
{
	reds_ = reds;
}

void yellow_cb(const oit_msgs::BlobArrayConstPtr& yellows )
{
	yellows_ = yellows;
}

void blob_cb( const oit_msgs::BlobArrayConstPtr& blobs )
{
  std::vector<double> x, y;
  std::vector<int> area;
 // std::vector<cv::Point3d> rays;

  // for each blobarray message
  if( model_initialized )
  {
		//ROS_INFO( "blobs.size: %d", blobs->blobs.size() );
    setHeightFromPublishedTransform();
    //ROS_INFO( "height: %0.2f cam_height: %0.2f", height, cam_height );
    // rectify blobs to real-world points
    
    tf::StampedTransform floor_to_cam, ray1_tf, ray2_tf;
		
	try
	{
		listener->waitForTransform("/ovh", "/ovh_height", ros::Time(0), ros::Duration(5.0));
		listener->lookupTransform("/ovh", "/ovh_height", ros::Time(0), floor_to_cam);
	}
	catch( tf::TransformException &ex )
	{
		ROS_WARN( "unable to do transformation: [%s]", ex.what());
	}
	
	double ir_plane = height - floor_to_cam.getOrigin().z();
	
	tf::Transform floor_to_cam_pub ( tf::Quaternion( 0, 0, 0), floor_to_cam.getOrigin() );
	
	tb->sendTransform( tf::StampedTransform(floor_to_cam_pub, blobs->header.stamp, "/ovh", "/cam_plane_flat" ) );
	
	try
	{
		listener->lookupTransform("/cam_plane_flat", "/ovh_height", ros::Time(0), floor_to_cam);
	}
	catch( tf::TransformException &ex )
	{
		ROS_WARN( "unable to do transformation: [%s]", ex.what());
	}
	
	/*tf::Vector3 ray1_rel_cam (ir1_ray.x, ir1_ray.y, ir1_ray.z);
	tf::Vector3 ray2_rel_cam (ir2_ray.x, ir2_ray.y, ir2_ray.z);
	
	tf::Vector3 ray1_uv ( floor_to_cam * ray1_rel_cam );
	tf::Vector3 ray2_uv ( floor_to_cam * ray2_rel_cam );*/
    
    for( int i = 0; i < blobs->blobs.size(); i++ )
    {
      oit_msgs::Blob blob = blobs->blobs[i];
      cv::Point2d uv;
      uv.x = blob.x + blob.width/2.0; uv.y = blob.y + blob.height/2.0;
      cv::Point2d uvrect;
      pcam.rectifyPoint( uv, uvrect );

      cv::Point3d imgray;
      
      pcam.projectPixelTo3dRay( uvrect, imgray );
      
      tf::Vector3 imgray_rel_cam (imgray.x, imgray.y, imgray.z);
      tf::Vector3 imgray_uv ( floor_to_cam * imgray_rel_cam );

      // project to correct distance from camera
      
      cv::Point3d proj_pt (ir_plane * imgray_uv.x() / imgray_uv.z(), ir_plane * imgray_uv.y() / imgray_uv.z(), ir_plane);

      //double fact = (cam_height-height)/imgray.z;
      //double px = fact*imgray.x; double py = -fact*imgray.y;
      x.push_back(proj_pt.x); y.push_back(proj_pt.y); area.push_back(blob.size);
      //rays.push_back(imgray); //save the original rays to the blobs
      
      //ROS_INFO("x %f y %f area %d", px, py, blob.size);
      
    }
    // figure out most likely correspondence
    double best_fit = 0;
    int n = blobs->blobs.size();
    double **fit = new double*[n];
    double irx1, iry1, irx2, iry2;
    cv::Point3d ir1_ray, ir2_ray;

    for( int i = 0; i < n; i++ )
    {
      fit[i] = new double[n];
      for( int j = 0; j < n; j++ ) fit[i][j] = 0;
    }

    for( int i = 0; i < n; i++ )
    {
      for( int j = 0; j < i; j++ )
      {
        double dist = hypot( y[i]-y[j], x[i]-x[j] );
        int small = area[i]>area[j] ? j : i;
        int large = area[i]<area[j] ? j : i;

        if( i == j ) fit[i][j] = 0;
        else if( fabs(dist - blob_dist_baseline) > blob_dist_tolerance ) fit[i][j] = 0;
        else if( area[large] < 10 || area[small] < 2 ) fit[i][j] = 0;
        else
        {
          fit[i][j] = area[large] + area[small];
        }

        //ROS_INFO( "(%0.2f,%0.2f)[%d] (%0.2f, %0.2f)[%d], %0.2f, %0.2f\n", x[i], y[i], area[i], x[j], y[j], area[j], dist, fit[i][j] );

        if( fit[i][j] > best_fit )
        {
          //ROS_INFO( "fit: %d %d", large, small);
          best_fit = fit[i][j];
          irx1 = x[large]; iry1 = y[large];
          irx2 = x[small]; iry2 = y[small];
          //ir1_ray = rays[large];
          //ir2_ray = rays[small];
        }
      }
    }
	
	//printf("best_fit: %f\n", best_fit);
	
    if( best_fit > 0 )
    {
		/*tf::StampedTransform floor_to_cam, ray1_tf, ray2_tf;
		
		try
		{
			listener->waitForTransform("/ovh", "/ovh_height", ros::Time(0), ros::Duration(5.0));
			listener->lookupTransform("/ovh", "/ovh_height", ros::Time(0), floor_to_cam);
		}
		catch( tf::TransformException &ex )
		{
			ROS_WARN( "unable to do transformation: [%s]", ex.what());
		}
		
		double ir_plane = height - floor_to_cam.getOrigin().z();
		
		tf::Transform floor_to_cam_pub ( tf::Quaternion( 0, 0, 0), floor_to_cam.getOrigin() );
		
		tb->sendTransform( tf::StampedTransform(floor_to_cam_pub, blobs->header.stamp, "/ovh", "/cam_plane_flat" ) );
		
		try
		{
			listener->lookupTransform("/cam_plane_flat", "/ovh_height", ros::Time(0), floor_to_cam);
		}
		catch( tf::TransformException &ex )
		{
			ROS_WARN( "unable to do transformation: [%s]", ex.what());
		}
		
		tf::Vector3 ray1_rel_cam (ir1_ray.x, ir1_ray.y, ir1_ray.z);
		tf::Vector3 ray2_rel_cam (ir2_ray.x, ir2_ray.y, ir2_ray.z);
		tf::Vector3 ray1_uv ( floor_to_cam * ray1_rel_cam );
		tf::Vector3 ray2_uv ( floor_to_cam * ray2_rel_cam );*/
		
		//ROS_INFO("r1uv x %f y %f z %f", ray1_uv.x(), ray1_uv.y(), ray1_uv.z() );
		//ROS_INFO("r2uv x %f y %f z %f", ray2_uv.x(), ray2_uv.y(), ray2_uv.z() );
		
		/*try
		{
			listener->waitForTransform("/cam_plane_flat", "/ray1_rel_cam", ros::Time(0), ros::Duration(5.0));
			listener->lookupTransform("/cam_plane_flat", "/ray1_rel_cam", ros::Time(0), ray1_tf);
			
			listener->waitForTransform("/cam_plane_flat", "/ray1_rel_cam", ros::Time(0), ros::Duration(5.0));
			listener->lookupTransform("/cam_plane_flat", "/ray2_rel_cam", ros::Time(0), ray2_tf);
		}
		catch( tf::TransformException &ex )
		{
			ROS_WARN( "unable to do transformation: [%s]", ex.what());
		}
		
		tf::Vector3 ray1_uv ( ray1_tf.getOrigin() );
		tf::Vector3 ray2_uv ( ray2_tf.getOrigin() );*/
		
		//x = at + x'; y = bt + y'; z = ct + z'
		//z = ir_plane = height
		//x' = y' = z' = 0; ray starts at center of camera
		//t = z / c
		//x = z * a / c
		//y = z * b / c
		
		//cv::Point3d ir1 (ir_plane * ray1_uv.x() / ray1_uv.z(), ir_plane * ray1_uv.y() / ray1_uv.z(), ir_plane);
		//cv::Point3d ir2 (ir_plane * ray2_uv.x() / ray2_uv.z(), ir_plane * ray2_uv.y() / ray2_uv.z(), ir_plane);
		//cv::Point3d irc ((ir1.x + ir2.x) / 2.0, (ir1.y + ir2.y) / 2.0, ir_plane);
		
		// determine yaw
		double yaw = atan2( iry1-iry2, irx1-irx2 ) + M_PI /2.0;
		tf::Quaternion quat; quat.setRPY( 0, 0, yaw );
		cv::Point3d irc ((irx1 + irx2) / 2.0, (iry1 + iry2) / 2.0, ir_plane);
		
		tf::Transform ir1_tf (quat, tf::Vector3(irx1, iry1, ir_plane) );
		tf::Transform ir2_tf (quat, tf::Vector3(irx2, iry2, ir_plane) );
		tf::Transform irc_tf (quat, tf::Vector3(irc.x, irc.y, ir_plane) );
		tb->sendTransform( tf::StampedTransform(ir1_tf, blobs->header.stamp, "/cam_plane_flat", "/robot/ir1" ) );
		tb->sendTransform( tf::StampedTransform(ir2_tf, blobs->header.stamp, "/cam_plane_flat", "/robot/ir2" ) );
		tb->sendTransform( tf::StampedTransform(irc_tf, blobs->header.stamp, "/cam_plane_flat", "/robot/ir" ) );
    }
  }
  else
  {
	  //printf("camera model not initialized\n");
    ROS_WARN( "no camera model input, ignoring blob message" );
  }
} 

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "pr2_finder" );
  tb = new tf::TransformBroadcaster();
  listener = new tf::TransformListener();
  ros::NodeHandle nh;
  nh.param("blob_dist_baseline", blob_dist_baseline, 0.3);
  nh.param("blob_dist_tolerance", blob_dist_tolerance, 0.25);
  ros::Subscriber yellow_sub = nh.subscribe( "yellow_blobs", 1, &yellow_cb );
  ros::Subscriber red_sub = nh.subscribe( "red_blobs", 1, &red_cb );
  ros::Subscriber camera_sub = nh.subscribe( "camera_info", 1, &camera_cb );
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("image", 1, image_cb );

	ros::Rate loop_rate(30);

	ros::Time last_time = ros::Time::now();

	cvNamedWindow("pr2_finder",0);


	while( ros::ok() )
	{
		loop_rate.sleep();
		ros::spinOnce();
		if( !yellows_ || !reds_ ) continue;
		// find biggest yellow blob
		if( yellows_->header.stamp > last_time && (reds_->header.stamp - yellows_->header.stamp).toSec() < 0.03 )
		{
			last_time = yellows_->header.stamp;
			// if new data
			
			int max_area = 0;
			oit_msgs::Blob yb;

			for( int i = 0; i < yellows_->blobs.size(); i++ )
			{
				if( yellows_->blobs[i].size > max_area )
				{
					max_area = yellows_->blobs[i].size;
					yb = yellows_->blobs[i];
				}
			}

			double min_dist = 999;
			oit_msgs::Blob rb;

			// find nearby red dot
			for( int i = 0; i < reds_->blobs.size(); i++ )
			{
				oit_msgs::Blob b = reds_->blobs[i];
				double d = hypot( (b.x+b.width/2)-(yb.x-yb.width/2), (b.y+b.height/2)-(yb.y+yb.height/2) );
				if( d < min_dist )
				{
					min_dist = d;
					rb = b;
				}
			}

			cvCircle( frame, cvPoint( yb.x+yb.width/2, yb.y+yb.height/2 ), 5, CV_RGB(0,0,255), -1 );
			cvCircle( frame, cvPoint( rb.x+rb.width/2, rb.y+rb.height/2 ), 3, CV_RGB(0,255,0), -1 );

			cvShowImage("pr2_finder",frame );
			cvWaitKey(10);
	

		  if( model_initialized )
  		{
				//ROS_INFO( "blobs.size: %d", blobs->blobs.size() );
	    	setHeightFromPublishedTransform();
	  	  //ROS_INFO( "height: %0.2f cam_height: %0.2f", height, cam_height );
  	  	// rectify blobs to real-world points
    
	  	  tf::StampedTransform floor_to_cam, ray1_tf, ray2_tf;
		
				try
				{
					listener->waitForTransform("/ovh", "/ovh_height", ros::Time(0), ros::Duration(0.250));
					listener->lookupTransform("/ovh", "/ovh_height", ros::Time(0), floor_to_cam);
				}
				catch( tf::TransformException &ex )
				{
					ROS_WARN( "unable to do transformation: [%s]", ex.what());
					continue;
				}
	
				double ir_plane = height - floor_to_cam.getOrigin().z();
	
				tf::Transform floor_to_cam_pub ( tf::Quaternion( 0, 0, 0), floor_to_cam.getOrigin() );
	
				tb->sendTransform( tf::StampedTransform(floor_to_cam_pub, yellows_->header.stamp, "/ovh", "/cam_plane_flat" ) );
	
				try
				{
					listener->lookupTransform("/cam_plane_flat", "/ovh_height", ros::Time(0), floor_to_cam);
				}
				catch( tf::TransformException &ex )
				{
					ROS_WARN( "unable to do transformation: [%s]", ex.what());
					continue;
				}

				// get position of red dot 
				cv::Point2d uv;
  	    uv.x = rb.x + rb.width/2.0; uv.y = rb.y + rb.height/2.0;
	      cv::Point2d uvrect;
      	pcam.rectifyPoint( uv, uvrect );

    	  cv::Point3d imgray;
      
  	    pcam.projectPixelTo3dRay( uvrect, imgray );
      
	      tf::Vector3 imgray_rel_cam (imgray.x, imgray.y, imgray.z);
      	tf::Vector3 imgray_uv ( floor_to_cam * imgray_rel_cam );

    	  // project to correct distance from camera
  	    cv::Point3d proj_pt (ir_plane * imgray_uv.x() / imgray_uv.z(), ir_plane * imgray_uv.y() / imgray_uv.z(), ir_plane);
				// x,y is proj_pt.x, proj_pt.y

				tf::Transform robot_pub;
				ROS_INFO( "height: %0.2f floor_to_cam: %0.2f ir_plane: %0.2f", height, floor_to_cam.getOrigin().z(), ir_plane );
				robot_pub.setOrigin( tf::Vector3(proj_pt.x, proj_pt.y, ir_plane ));//floor_to_cam.getOrigin().z()) );
				robot_pub.setRotation( tf::Quaternion(0,0,0));
				tb->sendTransform(tf::StampedTransform(robot_pub, yellows_->header.stamp, "cam_plane_flat", "red_dot" ));

			}
		}
		else if( last_time > yellows_->header.stamp )
		{
			last_time = yellows_->header.stamp;
		}
	
	}
  return 0;
}

