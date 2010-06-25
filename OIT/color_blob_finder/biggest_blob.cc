#include <ros/ros.h>
#include <oit_msgs/BlobArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// need pinhole camera model
bool model_initialized = false;
double height = 0.0;//0.812;
double cam_height = 0.0;
std::string tfname;
image_geometry::PinholeCameraModel pcam;
tf::TransformBroadcaster* tb;
tf::TransformListener* listener;

void setHeightFromPublishedTransform()
{
  tf::StampedTransform height_tf, cam_height_tf;
  try
  {
    listener->lookupTransform("/ovh_height", "/ovh", ros::Time(0), cam_height_tf);
  }
  catch (tf::TransformException ex)
  {
      ROS_ERROR("%s",ex.what());
      return;
  }
  cam_height = cam_height_tf.getOrigin().z();
}


void camera_cb( const sensor_msgs::CameraInfoConstPtr& cam )
{
  if( !model_initialized )
  {
    pcam.fromCameraInfo( cam );
    model_initialized = true;
  }
}

void blob_cb( const oit_msgs::BlobArrayConstPtr& blobs )
{
  // for each blobarray message
  if( model_initialized )
  {
    setHeightFromPublishedTransform();
    int max_area = 0;
    int idx = -1;
    for( int i = 0; i < blobs->blobs.size(); i++ )
    {
      if( blobs->blobs[i].size > max_area )
      {
        idx = i;
        max_area = blobs->blobs[i].size;
      }
    }

    if( idx > -1 )
    {
      oit_msgs::Blob blob = blobs->blobs[idx];
      cv::Point2d uv;
      uv.x = blob.x + blob.width/2.0; uv.y = blob.y + blob.height/2.0;
      cv::Point2d uvrect;
      pcam.rectifyPoint( uv, uvrect );

      cv::Point3d imgray;
      pcam.projectPixelTo3dRay( uvrect, imgray );

      // project to correct distance from camera
      double fact = (cam_height-height)/imgray.z;
      double px = fact*imgray.x; double py = -fact*imgray.y;

      // publish transform
      tf::Quaternion quat; quat.setRPY(0,0,0);
      tf::Transform blobt( quat, tf::Point( px, py, height ));
      tb->sendTransform( tf::StampedTransform(blobt, blobs->header.stamp, "/ovh", tfname ));
    }
  }
  else
  {
    ROS_WARN( "no camera model input, ignoring blob message" );
  }
}

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "blob_finder" );
  tb = new tf::TransformBroadcaster();
  listener = new tf::TransformListener();
  ros::NodeHandle nh;
  ros::Subscriber blobs_sub = nh.subscribe( "child_blobs", 1, &blob_cb );
  ros::Subscriber camera_sub = nh.subscribe( "camera_info", 1, &camera_cb );
  nh.param( "tfname", tfname, std::string("/child/odom") );
  nh.param( "height", height, 1.0 );
  ros::spin();
  return 0;
}

