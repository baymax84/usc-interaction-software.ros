/*
 * publish_topics.cpp
 *
 *  Created on: Dec 3, 2010
 *      Author: Christian Potthast
 */

#include <nbv_main/publish_topics.h>

PublishTopics::PublishTopics(ros::NodeHandle& n, std::string tf_frame)
{
  node_handle_ = n;
  tf_frame_ = tf_frame;
  robotPositionIndex_ = -1;
}

void PublishTopics::newTopic(std::string &name, sensor_msgs::PointCloud2 pc)
{
  // create a new publisher and store it in a vector
  ros::Publisher publisher;
  publisher = node_handle_.advertise<sensor_msgs::PointCloud2> (name, 10);
  publisher_.push_back(publisher);

  // save the cloud message which is suppose to be published
  cloud_msg_.push_back(pc);
}

void PublishTopics::newVisualizationMarker(visualization_msgs::Marker& marker_msg)
{
  bool exist = false;
  // check if marker already exist
  // if yes, update, otherwise create new one
  for(unsigned int ii=0; ii<marker_msg_.size(); ii++ )
  {

    if( strcmp(marker_msg.ns.c_str(), marker_msg_[ii].ns.c_str()) == 0)
    {
      exist = true;
      marker_msg_[ii] = marker_msg;
    }
  }

  if(exist == false)
  {
    ros::Publisher publisher;
    publisher = node_handle_.advertise<visualization_msgs::Marker>("NBV", 15);
    vis_publisher_.push_back(publisher);

    marker_msg_.push_back(marker_msg);
  }
}

void PublishTopics::samplingPositionsMsg(std::vector<Eigen::Vector3f>& sampling_positions)
{
  visualization_msgs::Marker sampling_position_msg;

  sampling_position_msg.header.frame_id = tf_frame_;
  sampling_position_msg.header.stamp = ros::Time::now();
  sampling_position_msg.ns = "Sampling_Positions";
  sampling_position_msg.id = 0;
  sampling_position_msg.type = visualization_msgs::Marker::SPHERE_LIST;
  sampling_position_msg.action = visualization_msgs::Marker::ADD;

  sampling_position_msg.scale.x = 0.03;
  sampling_position_msg.scale.y = 0.03;
  sampling_position_msg.scale.z = 0.03;

  sampling_position_msg.color.r = 0.0;
  sampling_position_msg.color.g = 1.0;
  sampling_position_msg.color.b = 0.0;
  sampling_position_msg.color.a = 1.0;

  sampling_position_msg.pose.orientation.x = 0.0;
  sampling_position_msg.pose.orientation.y = 0.0;
  sampling_position_msg.pose.orientation.z = 0.0;
  sampling_position_msg.pose.orientation.w = 1.0;

  geometry_msgs::Point p;
  for(unsigned int ii=0; ii<(unsigned int)sampling_positions.size(); ii++)
  {
    p.x = sampling_positions[ii].x();
    p.y = sampling_positions[ii].y();
    p.z = sampling_positions[ii].z();

    sampling_position_msg.points.push_back(p);
  }

  newVisualizationMarker(sampling_position_msg);

}

void PublishTopics::arrowMsg(std::string name, geometry_msgs::PoseStamped pose)
{
  visualization_msgs::Marker arrow_msg;

  arrow_msg.header.frame_id = tf_frame_;
  arrow_msg.header.stamp = ros::Time::now();
  arrow_msg.ns = name;
  arrow_msg.id = 0;
  arrow_msg.type = visualization_msgs::Marker::ARROW;
  arrow_msg.action = visualization_msgs::Marker::ADD;

  arrow_msg.scale.x = 0.5;
  arrow_msg.scale.y = 0.5;
  arrow_msg.scale.z = 0.5;

  arrow_msg.color.r = 0.0;
  arrow_msg.color.g = 1.0;
  arrow_msg.color.b = 0.0;
  arrow_msg.color.a = 1.0;

  arrow_msg.pose.position = pose.pose.position;
  arrow_msg.pose.orientation = pose.pose.orientation;

  newVisualizationMarker(arrow_msg);
}


void PublishTopics::robotPositionsMsg(Eigen::Vector3f robot_pose)
{
  if(robotPositionIndex_ == -1)
  {
    visualization_msgs::Marker robot_position_msg;

    robot_position_msg.header.frame_id = tf_frame_;
    robot_position_msg.header.stamp = ros::Time::now();
    robot_position_msg.ns = "Robot_Positions";
    robot_position_msg.id = 0;
    robot_position_msg.type = visualization_msgs::Marker::SPHERE_LIST;
    robot_position_msg.action = visualization_msgs::Marker::ADD;

    robot_position_msg.scale.x = 0.03;
    robot_position_msg.scale.y = 0.03;
    robot_position_msg.scale.z = 0.03;

    robot_position_msg.color.r = 1.0;
    robot_position_msg.color.g = 0.0;
    robot_position_msg.color.b = 0.0;
    robot_position_msg.color.a = 1.0;

    robot_position_msg.pose.orientation.x = 0.0;
    robot_position_msg.pose.orientation.y = 0.0;
    robot_position_msg.pose.orientation.z = 0.0;
    robot_position_msg.pose.orientation.w = 1.0;

    geometry_msgs::Point p;
    p.x = robot_pose.x();
    p.y = robot_pose.y();
    p.z = robot_pose.z();
    robot_position_msg.points.push_back(p);

    newVisualizationMarker(robot_position_msg);
    robotPositionIndex_ = marker_msg_.size() - 1;

  }else{
    geometry_msgs::Point p;
    p.x = robot_pose.x();
    p.y = robot_pose.y();
    p.z = robot_pose.z();
    marker_msg_[robotPositionIndex_].points.push_back(p);
  }

}



void PublishTopics::centroidPositionsMsg(Eigen::Vector4f& centroid)
{
  visualization_msgs::Marker centroid_position_msg;

  centroid_position_msg.header.frame_id = tf_frame_;
  centroid_position_msg.header.stamp = ros::Time::now();
  centroid_position_msg.ns = "Centroid_Positions";
  centroid_position_msg.id = 0;
  centroid_position_msg.type = visualization_msgs::Marker::SPHERE;
  centroid_position_msg.action = visualization_msgs::Marker::ADD;

  centroid_position_msg.scale.x = 0.02;
  centroid_position_msg.scale.y = 0.02;
  centroid_position_msg.scale.z = 0.02;

  centroid_position_msg.color.r = 0.0;
  centroid_position_msg.color.g = 1.0;
  centroid_position_msg.color.b = 0.0;
  centroid_position_msg.color.a = 1.0;

  centroid_position_msg.pose.orientation.x = 0.0;
  centroid_position_msg.pose.orientation.y = 0.0;
  centroid_position_msg.pose.orientation.z = 0.0;
  centroid_position_msg.pose.orientation.w = 1.0;

  centroid_position_msg.pose.position.x = centroid.x();
  centroid_position_msg.pose.position.y = centroid.y();
  centroid_position_msg.pose.position.z = centroid.z();

  newVisualizationMarker(centroid_position_msg);

}

void PublishTopics::voxelGridBbxMsg(std::vector<Eigen::Vector3f> table_top_bbx, Eigen::Vector3f bbx)
{

  // draw the voxel grid bounding box
  visualization_msgs::Marker voxel_grid_bbx_msg;

  voxel_grid_bbx_msg.header.frame_id = tf_frame_;
  voxel_grid_bbx_msg.header.stamp = ros::Time::now();
  voxel_grid_bbx_msg.ns = "Voxel_Grid_Bbx";
  voxel_grid_bbx_msg.id = 0;
  voxel_grid_bbx_msg.type = visualization_msgs::Marker::LINE_STRIP;
  voxel_grid_bbx_msg.action = visualization_msgs::Marker::ADD;

  voxel_grid_bbx_msg.scale.x = 0.005;

  voxel_grid_bbx_msg.color.r = 1.0;
  voxel_grid_bbx_msg.color.a = 1.0;

  geometry_msgs::Point p;
  float z = 0.0;
  for(unsigned int ii=0; ii<2; ii++)
  {
    if(ii==1)
      z = bbx.z();

    p.x = table_top_bbx[0].x();
    p.y = table_top_bbx[0].y();
    p.z = table_top_bbx[0].z() + z;
    voxel_grid_bbx_msg.points.push_back(p);
    p.x = table_top_bbx[1].x();
    p.y = table_top_bbx[1].y();
    p.z = table_top_bbx[1].z() + z;
    voxel_grid_bbx_msg.points.push_back(p);
    p.x = table_top_bbx[2].x();
    p.y = table_top_bbx[2].y();
    p.z = table_top_bbx[2].z() + z;
    voxel_grid_bbx_msg.points.push_back(p);
    p.x = table_top_bbx[3].x();
    p.y = table_top_bbx[3].y();
    p.z = table_top_bbx[3].z() + z;
    voxel_grid_bbx_msg.points.push_back(p);
    p.x = table_top_bbx[0].x();
    p.y = table_top_bbx[0].y();
    p.z = table_top_bbx[0].z() + z;
    voxel_grid_bbx_msg.points.push_back(p);
  }

  p.x = table_top_bbx[1].x();
  p.y = table_top_bbx[1].y();
  p.z = table_top_bbx[1].z() + z;
  voxel_grid_bbx_msg.points.push_back(p);
  p.x = table_top_bbx[1].x();
  p.y = table_top_bbx[1].y();
  p.z = table_top_bbx[1].z();
  voxel_grid_bbx_msg.points.push_back(p);
  p.x = table_top_bbx[2].x();
  p.y = table_top_bbx[2].y();
  p.z = table_top_bbx[2].z();
  voxel_grid_bbx_msg.points.push_back(p);
  p.x = table_top_bbx[2].x();
  p.y = table_top_bbx[2].y();
  p.z = table_top_bbx[2].z() + z;
  voxel_grid_bbx_msg.points.push_back(p);
  p.x = table_top_bbx[3].x();
  p.y = table_top_bbx[3].y();
  p.z = table_top_bbx[3].z() + z;
  voxel_grid_bbx_msg.points.push_back(p);
  p.x = table_top_bbx[3].x();
  p.y = table_top_bbx[3].y();
  p.z = table_top_bbx[3].z();
  voxel_grid_bbx_msg.points.push_back(p);



  newVisualizationMarker(voxel_grid_bbx_msg);
}

void PublishTopics::tableTopBbxMsg(std::vector<Eigen::Vector3f>& table_top_bbx)
{
  // draw the table top bounding box
  visualization_msgs::Marker table_top_bbx_msg;

  table_top_bbx_msg.header.frame_id = tf_frame_;
  table_top_bbx_msg.header.stamp = ros::Time::now();
  table_top_bbx_msg.ns = "Table_Top_Bbx";
  table_top_bbx_msg.id = 0;
  table_top_bbx_msg.type = visualization_msgs::Marker::LINE_STRIP;
  table_top_bbx_msg.action = visualization_msgs::Marker::ADD;

  table_top_bbx_msg.scale.x = 0.005;

  table_top_bbx_msg.color.r = 1.0;
  table_top_bbx_msg.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = table_top_bbx[0].x();
  p.y = table_top_bbx[0].y();
  p.z = table_top_bbx[0].z();
  table_top_bbx_msg.points.push_back(p);
  p.x = table_top_bbx[1].x();
  p.y = table_top_bbx[1].y();
  p.z = table_top_bbx[1].z();
  table_top_bbx_msg.points.push_back(p);
  p.x = table_top_bbx[2].x();
  p.y = table_top_bbx[2].y();
  p.z = table_top_bbx[2].z();
  table_top_bbx_msg.points.push_back(p);
  p.x = table_top_bbx[3].x();
  p.y = table_top_bbx[3].y();
  p.z = table_top_bbx[3].z();
  table_top_bbx_msg.points.push_back(p);
  p.x = table_top_bbx[0].x();
  p.y = table_top_bbx[0].y();
  p.z = table_top_bbx[0].z();
  table_top_bbx_msg.points.push_back(p);

  newVisualizationMarker(table_top_bbx_msg);
}


void PublishTopics::Publish()
{
  for(unsigned int ii=0; ii<cloud_msg_.size(); ii++)
  {
    publisher_[ii].publish(cloud_msg_[ii]);
  }

  for(unsigned int ii=0; ii<marker_msg_.size(); ii++)
  {
    vis_publisher_[ii].publish(marker_msg_[ii]);
  }

}
