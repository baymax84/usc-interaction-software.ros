/*
 * save_from_topic.cpp
 *
 *  Created on: May 18, 2011
 *      Author: potthast
 */

#include <tools/save_from_topic.h>

SaveFromTopic::SaveFromTopic(ros::NodeHandle& n)
{
  node_handle_ = n;
  saved = false;
  tf_ = false;
}

void SaveFromTopic::subscribe(std::string& topic_name, bool rgb, bool tf)
{
  origin_.x() = 0.0;
  origin_.y() = 0.0;
  origin_.z() = 0.0;
  origin_.w() = 0.0;
  orientation_.x() = 0.0;
  orientation_.y() = 0.0;
  orientation_.z() = 0.0;
  orientation_.w() = 0.0;

  if(tf)
  {
    tf_ = true;

    tf::TransformListener listener;
    bool check = false;
    while(!check){
      // get the kinect_rgb_frame pose
      if (listener.waitForTransform("/base_link", tf_name, ros::Time(0), ros::Duration(0.01)))
      {
        //ROS_INFO("%f %f %f", transform_.getOrigin().x(), transform_.getOrigin().y(), transform_.getOrigin().z());
        listener.lookupTransform("/base_link", tf_name, ros::Time(0), transform_);
      }

      if(transform_.getOrigin().z() > 0.001)
        check = true;
    }

    origin_.x() = transform_.getOrigin().x();
    origin_.y() = transform_.getOrigin().y();
    origin_.z() = transform_.getOrigin().z();
    origin_.w() = transform_.getOrigin().w();
    orientation_.x() = transform_.getRotation().x();
    orientation_.y() = transform_.getRotation().y();
    orientation_.z() = transform_.getRotation().z();
    orientation_.w() = transform_.getRotation().w();

    ROS_INFO("tf: %f %f %f", origin_.x(), origin_.y(), origin_.z());
    ROS_INFO("tf: %f %f %f %f", orientation_.x(), orientation_.y(), orientation_.z(), orientation_.w());

  }


  if(rgb)
    sub_ = node_handle_.subscribe(topic_name, 1, &SaveFromTopic::getPointCloudRGB, this);
//  else if(tf)
//    sub_ = node_handle_.subscribe(topic_name, 1, &SaveFromTopic::getPointCloud, this);
  else
    sub_ = node_handle_.subscribe(topic_name, 1, &SaveFromTopic::getPointCloud, this);

}

tf::StampedTransform SaveFromTopic::getCloudTransform(std_msgs::Header& pc_header)
{
  tf::StampedTransform transform;
  tf::TransformListener listener;

  bool check = false;
  int c=0;
  while(!check){
    c++;
    if (listener.waitForTransform("/base_link", pc_header.frame_id, ros::Time(0) ,ros::Duration(5)))
    {
      listener.lookupTransform( "/base_link", pc_header.frame_id, ros::Time(0), transform);
    }

    if(transform.getOrigin().z() > 0.001)
      check = true;
    if(c > 100)
      check = true;
  }
 // ROS_INFO("tf: %f %f %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
/*
  origin_.x() = transform.getOrigin().x();
  origin_.y() = transform.getOrigin().y();
  origin_.z() = transform.getOrigin().z();
  origin_.w() = transform.getOrigin().w();
  orientation_.x() = transform.getRotation().x();
  orientation_.y() = transform.getRotation().y();
  orientation_.z() = transform.getRotation().z();
  orientation_.w() = transform.getRotation().w();
*/

  return transform;
}

void SaveFromTopic::getPointCloud(const PointCloud::ConstPtr& msg)
{
  printf ("PointXYZ\n");
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

  PointCloud cloud = *msg;

  if(cloud.points.size() > 1)
  {
    if(tf_){
      tf::StampedTransform transform = getCloudTransform(cloud.header);
      pcl_ros::transformPointCloud(cloud, cloud, transform);
    }

    cloud.sensor_origin_ = origin_;
    cloud.sensor_orientation_ = orientation_;

    pcl::io::savePCDFileASCII (file_name , cloud);

    saved = true;
    sub_.shutdown();
  }
}


void SaveFromTopic::getPointCloudRGB(const PointCloudRGB::ConstPtr& msg)
{
  printf ("PointXYZRGB\n");
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

  PointCloudRGB cloud = *msg;

  if(cloud.points.size() > 1)
  {
    if(tf_){
      tf::StampedTransform transform = getCloudTransform(cloud.header);
      pcl_ros::transformPointCloud(cloud, cloud, transform);
    }

    cloud.sensor_origin_ = origin_;
    cloud.sensor_orientation_ = orientation_;

    pcl::io::savePCDFileASCII (file_name , cloud);

    saved = true;
    sub_.shutdown();
  }
}

/*
void SaveFromTopic::getPointCloudMsg(const sensor_msgs::PointCloud2 &msg)
{
  printf ("sensor_msgs::PointCloud2\n");
  printf ("Cloud: width = %d, height = %d\n", msg.width, msg.height);

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(msg, pcl_cloud);

  if(pcl_cloud.points.size() > 1)
  {
    pcl::io::savePCDFileASCII (file_name , pcl_cloud);
    saved = true;
    sub_.shutdown();
  }
}
*/

int main(int argc, char** argv)
{
  ros::init (argc, argv, "save_from_topic");
  ros::NodeHandle nh;
  SaveFromTopic save_from_topic(nh);

  if(argc < 3)
  {
    ROS_ERROR ("No input file specified");
    ROS_ERROR ("e.g.  save_from_topic -rgb /camera/rgb/points /openni_rgb_frame output.pcd");
    return (-1);
  }

  bool rgb=false;
  int index = 1;
  if(strcmp(argv[index], "-rgb") == 0){
    rgb=true;
    index++;
  }

  std::string topic_name = argv[index];


  bool tf = false;
  if( (rgb && argc > 4) || (!rgb && argc >3) ){
    std::string tf_name = argv[index+1];
    tf = true;
    save_from_topic.setTfName(tf_name);
    index++;
  }

  std::string file_name = argv[index+1];

  save_from_topic.subscribe(topic_name, rgb, tf);
  save_from_topic.setFileName(file_name);


  ros::Rate loop_rate(1);
  while (nh.ok())
  {
    if(save_from_topic.saved)
      ros::shutdown();
    ros::spinOnce ();
    loop_rate.sleep ();
  }

  return 0;
}
