#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <people_msgs/PositionMeasurement.h>

#include <tf/transform_listener.h>

#include <opencv/ml.h>

class HeadFollowPeople {

public:
  HeadFollowPeople( ros::NodeHandle nh ) : nh_(nh)
  {
    people_sub_ = nh_.subscribe("people_tracker_measurements", 1000, (boost::function< void(const people_msgs::PositionMeasurementConstPtr&)>) boost::bind(&HeadFollowPeople::people_cb, this, _1));

    head_target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("head_goal",1000);
    target_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("target_cloud", 1000);

    nh_.param( "base_frame", base_frame_, std::string("/ovh") );
    tracked_people_.empty();
    tl_ = new tf::TransformListener();
    rng_state_ = cvRNG(-1);
  }
  
  void people_cb( const people_msgs::PositionMeasurementConstPtr &meas )
  {
    // add to list (replace old member if needed)
    bool found = false;

    for( unsigned int i = 0; i < tracked_people_.size(); i++ )
    {
      if( tracked_people_[i].name == meas->name )
      {
        tracked_people_[i] = *meas;
        found = true;
      }
    }

    if( !found )
    {
      tracked_people_.push_back( *meas );
    }
  }

  void train_head()
  {
     // build rand array
    int nsamples = 500;
    CvMat* samples = cvCreateMat( nsamples, 3, CV_32FC1 );


    std::vector<geometry_msgs::PointStamped> points;

    //clear out old data
    for( unsigned int i = 0; i < tracked_people_.size(); i++ )
    {
      ROS_INFO( "%0.2f", (ros::Time::now() - tracked_people_[i].header.stamp).toSec() );
      if( (ros::Time::now() - tracked_people_[i].header.stamp) > ros::Duration( 0.5 ) )
      {
        tracked_people_.erase( tracked_people_.begin()+i );
      }
      else {
        // this is a valid point test if can be transformed
        try {
          geometry_msgs::PointStamped point;
          point.header = tracked_people_[i].header;
          point.point = tracked_people_[i].pos;
          tl_->transformPoint( base_frame_, point, point );
          points.push_back( point );
        }
        catch( tf::TransformException &ex )
        {
          ROS_WARN( "transform exception: [%s]", ex.what() );
        }

      }
    }

    int N = points.size();
    ROS_INFO( "N: %d", N );
    if( N <= 0 ) return;

    cvReshape (samples, samples, 3, 0 );
    CvMat samples_part;
    for( unsigned int i = 0; i < N; i++ )
    {
      CvScalar mean, sigma;
      cvGetRows( samples, &samples_part, i*nsamples/N, (i+1)*nsamples/N );
      mean = cvScalar(points[i].point.x,points[i].point.y,points[i].point.z);
      sigma = cvScalar(0.125,0.25,0.05);
      cvRandArr( &rng_state_, &samples_part, CV_RAND_NORMAL, mean, sigma );
    }
    cvReshape (samples, samples, 1, 0 );

    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = base_frame_;

    for( int i = 0; i < nsamples; i++ )
    {
      geometry_msgs::Point32 point;
      point.x = samples->data.fl[i*3];      
      point.y = samples->data.fl[i*3+1];      
      point.z = samples->data.fl[i*3+2];
      cloud.points.push_back(point);
    }

    // pick one of the random points
    unsigned int idx = cvRandInt(&rng_state_) % nsamples;
    //unsigned int idx = 234;
    geometry_msgs::PointStamped target_point;
    target_point.point.x = cloud.points[idx].x;
    target_point.point.y = cloud.points[idx].y;
    target_point.point.z = cloud.points[idx].z;
    target_point.header.frame_id = base_frame_;
    target_point.header.stamp = ros::Time::now();

    head_target_pub_.publish(target_point);
    target_cloud_pub_.publish(cloud);    
  }

  protected:
    ros::NodeHandle nh_;
    ros::Subscriber people_sub_;
    ros::Publisher head_target_pub_;
    ros::Publisher target_cloud_pub_;

    std::vector<people_msgs::PositionMeasurement> tracked_people_;
    tf::TransformListener *tl_;

    std::string base_frame_;

    CvRNG rng_state_;
};  

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "follow_poeple" );
  ros::NodeHandle nh;
  HeadFollowPeople hfp(nh);
  ros::Rate loop_rate(1);

  while( ros::ok() )
  {
    ros::spinOnce();
    hfp.train_head();
    loop_rate.sleep();
  }
  return 0;
}
