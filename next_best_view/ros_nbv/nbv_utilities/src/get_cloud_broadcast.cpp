/*
 * get_cloud_broadcast.cpp
 *
 *  Created on: Apr 12, 2011
 *      Author: potthast
 */

#include <nbv_utilities/get_cloud_broadcast.h>

GetCloudBroadcast::GetCloudBroadcast(ros::NodeHandle& n)
{
  node_handle_ = n;

  if(getCloudFromLaserScanner())
  {

    publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> ("LaserPointCloud", 10);

    // Start the timer that will trigger the processing loop (timerCallback)
    timer_ = node_handle_.createTimer(ros::Duration(5,0), &GetCloudBroadcast::timerCallback, this);
  }
}

bool GetCloudBroadcast::getCloudFromLaserScanner()
{
  nbv_utilities::PcdFile::Request req;
  nbv_utilities::PcdFile::Response res;

  req.req = true;

  if (!ros::service::call("nbv_utilities/PointCloud", req, res)){
    ROS_ERROR("GetCloudBroadcast : could not receive cloud from nbv_utilities/PointCloud");
    return false;

  }else{
    ROS_INFO("GetCloudBroadcast : Received cloud with size: %d",
             (int)(res.cloud.height * res.cloud.width));

  }

  cloud_ = res.cloud;

  return true;
}

void GetCloudBroadcast::timerCallback(const ros::TimerEvent& event)
{
  if(cloud_.data.size() > 0)
  {
    publisher_.publish(cloud_);
  }

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "get_cloud_broadcast");
  ros::NodeHandle n;

  GetCloudBroadcast getCloudBroadcast(n);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  //ros::spin();

  return 0;
}
