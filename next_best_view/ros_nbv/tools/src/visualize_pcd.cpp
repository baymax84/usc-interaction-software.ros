/*
 * visualize_pcd.cpp
 *
 *  Created on: May 18, 2011
 *      Author: potthast
 */

#include <tools/visualize_pcd.h>

VisualizePCD::VisualizePCD(ros::NodeHandle& n)
{
  tf_frame_  = ("/base_link");
  node_handle_ = n;
}

int VisualizePCD::load(std::string& file_name)
{
  sensor_msgs::PointCloud2 cloud;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;

  if (file_name == "" || pcl::io::loadPCDFile (file_name, cloud, origin, orientation) == -1)
    return (-1);

  // find return the field names of the cloud and split them
  std::string buf; // buffer string
  std::stringstream ss(pcl::getFieldsList(cloud)); // Insert the string into a stream
  std::vector<std::string> tokens; // Create vector to hold our words
  std::string fields;
  while (ss >> buf){
      fields = fields+buf;
      tokens.push_back(buf);
  }

  // create the topic name
  size_t index_pos_end;
  index_pos_end = file_name.find(".");
  std::string topic_name = file_name.substr(0, index_pos_end);

  // depending on the input type choose point cloud
  if( strcmp(fields.c_str(), "xyz") == 0 )
  {
    PointCloud point_cloud_xyz;
    pcl::fromROSMsg(cloud, point_cloud_xyz);
    point_cloud_xyz.sensor_origin_ = origin;
    point_cloud_xyz.sensor_orientation_ = orientation;
    point_cloud_xyz.header.stamp = ros::Time().now();
    point_cloud_xyz.header.frame_id = tf_frame_;

    pc_xyz_.push_back(point_cloud_xyz.makeShared());

    ros::Publisher pub = node_handle_.advertise<PointCloud> (topic_name, 1);
    publisher_xyz_.push_back(pub);

    vis_camera_direction(topic_name, origin, orientation);
  }
  else if( strcmp(fields.c_str(), "xyzrgb") == 0)
  {
    PointCloudRGB point_cloud_xyz_rgb;
    pcl::fromROSMsg(cloud, point_cloud_xyz_rgb);
    point_cloud_xyz_rgb.sensor_origin_ = origin;
    point_cloud_xyz_rgb.sensor_orientation_ = orientation;
    point_cloud_xyz_rgb.header.stamp = ros::Time().now();
    point_cloud_xyz_rgb.header.frame_id = tf_frame_;

    pc_xyz_rgb_.push_back(point_cloud_xyz_rgb.makeShared());

    ros::Publisher pub = node_handle_.advertise<PointCloudRGB> (topic_name, 1);
    publisher_xyz_rgb_.push_back(pub);

    vis_camera_direction(topic_name, origin, orientation);
  }else{
    return (-1);
  }

  std::cout << "-- file: " << file_name << " ; topic: " << topic_name <<  " ; points: " << cloud.width * cloud.height;
  std::cout << " ; channels: " << pcl::getFieldsList (cloud).c_str () << std::endl;

  return (0);
}

void VisualizePCD::vis_camera_direction(std::string name, Eigen::Vector4f origin, Eigen::Quaternionf orientation)
{
  visualization_msgs::Marker arrow_msg;

  arrow_msg.header.frame_id = tf_frame_;
  arrow_msg.header.stamp = ros::Time::now();
  arrow_msg.ns = name;
  arrow_msg.id = 0;
  arrow_msg.type = visualization_msgs::Marker::ARROW;
  arrow_msg.action = visualization_msgs::Marker::ADD;

  arrow_msg.scale.x = 0.2;
  arrow_msg.scale.y = 0.25;
  arrow_msg.scale.z = 0.25;

  arrow_msg.color.r = 1.0;
  arrow_msg.color.g = 0.0;
  arrow_msg.color.b = 0.0;
  arrow_msg.color.a = 1.0;

  arrow_msg.pose.position.x = origin.x();
  arrow_msg.pose.position.y = origin.y();
  arrow_msg.pose.position.z = origin.z();
  arrow_msg.pose.orientation.x = orientation.x();
  arrow_msg.pose.orientation.y = orientation.y();
  arrow_msg.pose.orientation.z = orientation.z();
  arrow_msg.pose.orientation.w = orientation.w();

  ros::Publisher pub = node_handle_.advertise<visualization_msgs::Marker>("Vis", 15);
  vis_.push_back(pub);

  marker_msg_.push_back(arrow_msg);
}

void VisualizePCD::publish()
{
  for(unsigned int ii=0; ii<publisher_xyz_.size(); ii++)
  {
    publisher_xyz_[ii].publish (pc_xyz_[ii]);
  }
  for(unsigned int ii=0; ii<publisher_xyz_rgb_.size(); ii++)
  {
    publisher_xyz_rgb_[ii].publish (pc_xyz_rgb_[ii]);
  }
  for(unsigned int ii=0; ii<vis_.size(); ii++)
  {
    vis_[ii].publish (marker_msg_[ii]);
  }
}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "visualize_pcl");
  ros::NodeHandle nh;
  VisualizePCD visualize_pcd(nh);

  if(argc < 2)
  {
    ROS_ERROR ("No input file specified");
    return (-1);
  }

  for(int ii=1; ii<argc; ii++)
  {
    std::string file_name = argv[ii];

    if (visualize_pcd.load(file_name) == -1)
    {
      ROS_ERROR ("Could not load file. Exiting.");
      return (-1);
    }
  }

  ros::Rate loop_rate(0.5);
  while (nh.ok())
  {
    visualize_pcd.publish();

    ros::spinOnce ();
    loop_rate.sleep ();
  }

  return 0;
}
