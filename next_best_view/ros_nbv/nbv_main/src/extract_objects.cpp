/*
 * extract_objects.cpp
 *
 *  Created on: Dec 2, 2010
 *      Author: Christian Potthast
 */

#include <nbv_main/extract_objects.h>


ExtractObjects::ExtractObjects(ros::NodeHandle& n)
{
  node_handle_ = n;
  utilities_ = Utilities();
  publish_topics_ = PublishTopics(node_handle_, "/map");

  // Service
  send_cloud_srv_ = node_handle_.advertiseService("extractObjects/SendCloud", &ExtractObjects::sendCloud, this);
  find_table_srv_ = node_handle_.advertiseService("extractObjects/FindTable", &ExtractObjects::findTable, this);
  kinect_table_srv_ = node_handle_.advertiseService("extractObjects/KinectCloud", &ExtractObjects::kinectCloud, this);

  // Start the timer that will trigger the processing loop (timerCallback)
  timer_ = node_handle_.createTimer(ros::Duration(5,0), &ExtractObjects::timerCallback, this);

  // THIS HAS TO BE CHANGE TO A MORE GENERAL WAY
  // define colors for objects
  colors_.push_back(utilities_.getRGB(1.0, 0.0, 0.0));
  colors_.push_back(utilities_.getRGB(0.0, 1.0, 0.0));
  colors_.push_back(utilities_.getRGB(0.0, 0.0, 1.0));
  colors_.push_back(utilities_.getRGB(1.0, 1.0, 0.0));
  colors_.push_back(utilities_.getRGB(0.0, 1.0, 1.0));
  colors_.push_back(utilities_.getRGB(1.0, 0.0, 1.0));

  init_ = false;
  direction_ = 0.0;

  //Tree search_tree;
  //Tree::node* root = search_tree.getRoot();
  //search_tree.unitTest();

}

void ExtractObjects::timerCallback(const ros::TimerEvent& event)
{
  publish_topics_.Publish();
}

bool ExtractObjects::sendCloud(nbv_utilities::SendCloud::Request &req,
                               nbv_utilities::SendCloud::Response &res)
{
  sensor_msgs::PointCloud2 cloud = req.cloud;
  if(req.dir != -1.0)
  {
    direction_ = req.dir;
  }

  // store the sensor direction into a file
  std::ifstream ifile("/home/potthast/projects/data/dir.txt");
  std::ofstream file;
  if((bool)ifile)
  {
    //open file
    file.open("/home/potthast/projects/data/dir.txt", std::ios::app);
  }else{
    //create a new file
    file.open("/home/potthast/projects/data/dir.txt");
  }
  file << (int)direction_ << "\n";
  file.close();



  int cloud_size = cloud.width * cloud.height;
  geometry_msgs::PoseWithCovarianceStamped robot_pose = req.robot_pose;
  Eigen::Vector3f scan_pos;
  scan_pos.x() =  robot_pose.pose.pose.position.x;
  scan_pos.y() =  robot_pose.pose.pose.position.y;
  scan_pos.z() =  robot_pose.pose.pose.position.z;
  scan_poses_.push_back(scan_pos);
  publish_topics_.robotPositionsMsg(scan_pos);

  ROS_INFO("ExtractObjects - sendCloud: size of the cloud: %d", cloud_size );
  ROS_INFO("ExtractObjects - sampling direction: %f", direction_ );

  // convert the point cloud2 msg into the pcl point cloud format
  pcl::PointCloud<Point> pcl_cloud;
  pcl::fromROSMsg (cloud, pcl_cloud);
  pcl::PointCloud<Point>::ConstPtr input_cloud_ptr;
  input_cloud_ptr = boost::make_shared<pcl::PointCloud<Point> >(pcl_cloud);
  input_clouds_.push_back(input_cloud_ptr);

  getTableTop(input_cloud_ptr, false);

  int index = input_clouds_.size() - 1 ;
  unsigned int init_case = 0;
  if(init_ == false)
    init_case = 1;
 // else
 //   init_case = 2;

  //ROS_INFO("init_case: %d ; size: %d", init_case, (int)input_clouds_.size());

  Eigen::Vector3f new_scan_position(0.0,0.0,0.0);
  if(input_clouds_.size() == init_case){
    // set the reference cloud
    pcl::PointCloud<Point> ref_cloud;
    for(unsigned int ii=0; ii<4; ii++)
    {
      Point point(table_top_bbx_[index][ii].x(),
                  table_top_bbx_[index][ii].y(),
                  table_top_bbx_[index][ii].z());
      ref_cloud.points.push_back(point);
    }
    // get the centroid of the bounding box
    table_top_centroid_.x() = table_top_bbx_[index][4].x();
    table_top_centroid_.y() = table_top_bbx_[index][4].y();
    table_top_centroid_.z() = table_top_bbx_[index][4].z();

    // add more points to the cloud
    for(unsigned int ii=0; ii<4; ii++)
    {
      Point p(ref_cloud.points[ii].x-0.05, ref_cloud.points[ii].y, ref_cloud.points[ii].z);
      ref_cloud.points.push_back(p);
      Point p2(ref_cloud.points[ii].x+0.05, ref_cloud.points[ii].y, ref_cloud.points[ii].z);
      ref_cloud.points.push_back(p2);
      Point p3(ref_cloud.points[ii].x, ref_cloud.points[ii].y-0.05, ref_cloud.points[ii].z);
      ref_cloud.points.push_back(p3);
      Point p4(ref_cloud.points[ii].x, ref_cloud.points[ii].y+0.05, ref_cloud.points[ii].z);
      ref_cloud.points.push_back(p4);
      Point p5(ref_cloud.points[ii].x, ref_cloud.points[ii].y, ref_cloud.points[ii].z-0.05);
      ref_cloud.points.push_back(p5);
      Point p6(ref_cloud.points[ii].x, ref_cloud.points[ii].y, ref_cloud.points[ii].z+0.05);
      ref_cloud.points.push_back(p6);
    }

    ref_cloud.header.frame_id="/map";
    reference_cloud_ = boost::make_shared<pcl::PointCloud<Point> >(ref_cloud);
    getObjects();
    //getObjectsLimitedFOV(15, "fov_1");
    getObjectsLimitedFOV(direction_, "fov_init");
   // getObjectsLimitedFOV(-15, "fov_3");

    labelObjects();

    initializeVoxelGrid();

    // initialize the planner
    planner_ = Planner(voxel_grid_, false, 1);
    planner_.setFullSamplingPositions(full_sampling_positions_);


  }else{
    transformClouds(index);
    getObjects();
    getObjectsLimitedFOV(direction_, "fov");
    labelObjects();

    ROS_INFO("Transform cloud - DONE");
  }

  setNewVoxelGridScan(labeled_objects_[labeled_objects_.size()-1]);
  const Eigen::Vector3f P0 = scan_poses_[scan_poses_.size()-1];
  //planner_.plan(P0, sampling_positions_);
  //bool state = planner_.plan_greedy(P0, sampling_positions_);
  //bool state = planner_.plan_expectation(P0, sampling_positions_);
  //bool state = planner_.plan_simple(P0, sampling_positions_);
  bool state = planner_.collect_scans(P0, sampling_positions_);
  //bool state = planner_.random_FOV(P0, full_sampling_positions_, sampling_angle_, direction_);
  //bool state = planner_.plan_simple_FOV(P0, full_sampling_positions_, sampling_angle_, direction_);
  //bool state = planner_.plan_expectation_FOV(P0, full_sampling_positions_, sampling_angle_, direction_);



  new_scan_position = planner_.getNewScanPosition();
  direction_ = planner_.getNewDirection();
  //ROS_INFO("%f %f %f", new_scan_position.x(), new_scan_position.y(), new_scan_position.z());
  publish_topics_.robotPositionsMsg(new_scan_position);

  //publish_topics_.robotPositionsMsg(sampling_positions_[5]);


  visualize();
  ROS_INFO("Visualize cloud - DONE");

  if(cloud_size > 0){
    double theta = atan2(table_top_centroid_.y() - new_scan_position.y(),
                           table_top_centroid_.x() - new_scan_position.x());

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose.position.x = new_scan_position.x();
    goal_pose.pose.position.y = new_scan_position.y();
    goal_pose.pose.position.z = new_scan_position.z();
    goal_pose.pose.orientation = odom_quat;


    std::string s = boost::lexical_cast<std::string>( (int)input_clouds_.size() );
    std::string topic_name = "arrow_" + s;
    publish_topics_.arrowMsg(topic_name, goal_pose);



    res.result = res.SUCCEEDED;
    res.goal_pose.pose = goal_pose.pose;
  }

  if(input_clouds_.size() == 11 || state == false)
    res.result = res.DONE;

  /*
    }else
      res.result = res.DONE;
  }else
    res.result = res.FAILED;
*/


  ROS_INFO("goal pose: %f %f ", res.goal_pose.pose.position.x, res.goal_pose.pose.position.y);

  return true;
}

bool ExtractObjects::findTable(nbv_utilities::FindTable::Request &req,
                               nbv_utilities::FindTable::Response &res)
{
  init_ = true;
  int n_sampling_positions = 10;

  sensor_msgs::PointCloud2 cloud = req.cloud;
  geometry_msgs::PoseWithCovarianceStamped robot_pose = req.robot_pose;
  Eigen::Vector3f scan_pos;
  scan_pos.x() =  robot_pose.pose.pose.position.x;
  scan_pos.y() =  robot_pose.pose.pose.position.y;
  scan_pos.z() =  robot_pose.pose.pose.position.z;
  scan_poses_.push_back(scan_pos);
  publish_topics_.robotPositionsMsg(scan_pos);

  int cloud_size = cloud.width * cloud.height;
  ROS_INFO("size of the cloud: %d", cloud_size );

  // convert the point cloud2 msg into the pcl point cloud format
  pcl::PointCloud<Point> pcl_cloud;
  pcl::fromROSMsg (cloud, pcl_cloud);
  pcl::PointCloud<Point>::ConstPtr input_cloud_ptr;
  input_cloud_ptr = boost::make_shared<pcl::PointCloud<Point> >(pcl_cloud);
  input_clouds_.push_back(input_cloud_ptr);

  // create a new publishing object
  std::string topic_name = "init_scan/input_cloud";
  cloud.header.frame_id = "/map";
  publish_topics_.newTopic(topic_name, cloud);

  getTableTop(input_cloud_ptr, false);
  getTableTop(input_cloud_ptr, true);


  int index = table_tops_.size() - 1;
  topic_name = "init_scan";
  //publish the table top point cloud
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*table_tops_[index], msg);
  std::string ss = topic_name + "/table_top";
  publish_topics_.newTopic(ss, msg);

  // publish the inverse of the table top point cloud
  pcl::toROSMsg(*inv_table_tops_[index], msg);
  ss = topic_name + "/inv_table_top";
  publish_topics_.newTopic(ss, msg);

  // get the centroid of the table top from the fitted rect of the table top point cloud
  Eigen::Vector4f centroid;
  centroid.x() = table_top_bbx_[0][4].x();
  centroid.y() = table_top_bbx_[0][4].y();
  centroid.z() = table_top_bbx_[0][4].z();

  publish_topics_.centroidPositionsMsg(centroid);

  samplingPositions(n_sampling_positions, robot_pose.pose.pose.position.z, centroid, 1.28);
  full_sampling_positions_ = sampling_positions_;


  // find the closest sampling position from where the robot is right now
  int sp_index = utilities_.getClosestPositionAndErase(sampling_positions_, robot_pose);

  sp_index = 0;

  // change this so that the robot is driving a little bit closer for the first scan.
  std::vector<Eigen::Vector3f> samplingPositions;
  samplingPositions = samplingPositions2(n_sampling_positions, robot_pose.pose.pose.position.z, centroid, 1.4);
  full_sampling_positions_[sp_index] = samplingPositions[sp_index];


  Eigen::Vector3f robot_pos = samplingPositions[sp_index];


  double theta = atan2(centroid.y() - robot_pos.y(),
                       centroid.x() - robot_pos.x());

  //double theta = atan2(   centroid.y() - robot_pose.pose.pose.position.y,
  //                        centroid.x() - robot_pose.pose.pose.position.x );

  //double theta = atan2(table_top_centroid_.y() - new_scan_position.y(),
  //                       table_top_centroid_.x() - new_scan_position.x());


//  ROS_INFO("x: %f - y: %f",sampling_positions_[sp_index].x(),sampling_positions_[sp_index].y() );
//  ROS_INFO("x: %f - y: %f",centroid.x(), centroid.y() );
//  ROS_INFO("theta: %f", theta*180/3.14);

 // theta = 0.0; //180.0*3.14/180;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.pose.position.x = robot_pos.x();
  goal_pose.pose.position.y = robot_pos.y();
  goal_pose.pose.position.z = robot_pos.z();
  goal_pose.pose.orientation = odom_quat;

/*
  // draw direction arrows
  for(int ii=0; ii<(int)full_sampling_positions_.size(); ii++)
  {
    std::string s = boost::lexical_cast<std::string>( ii );
      std::string topic_name = "arrow_" + s;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = full_sampling_positions_[ii].x();
    pose.pose.position.y = full_sampling_positions_[ii].y();
    pose.pose.position.z = full_sampling_positions_[ii].z();

    double theta = atan2(centroid.y() - full_sampling_positions_[ii].y(),
                         centroid.x() - full_sampling_positions_[ii].x());
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);

    pose.pose.orientation.x = quat.x;
    pose.pose.orientation.y = quat.y;
    pose.pose.orientation.z = quat.z;
    pose.pose.orientation.w = quat.w;

    publish_topics_.arrowMsg(topic_name, pose);
  }
*/






  pcl::PointCloud<Point> ref_cloud;
  for(unsigned int ii=0; ii<4; ii++)
  {
    Point point(table_top_bbx_[index][ii].x(),
                table_top_bbx_[index][ii].y(),
                table_top_bbx_[index][ii].z());
    ref_cloud.points.push_back(point);
  }
  // get the centroid of the bounding box
  table_top_centroid_.x() = table_top_bbx_[index][4].x();
  table_top_centroid_.y() = table_top_bbx_[index][4].y();
  table_top_centroid_.z() = table_top_bbx_[index][4].z();

  // add more points to the cloud
  for(unsigned int ii=0; ii<4; ii++)
  {
    Point p(ref_cloud.points[ii].x-0.05, ref_cloud.points[ii].y, ref_cloud.points[ii].z);
    ref_cloud.points.push_back(p);
    Point p2(ref_cloud.points[ii].x+0.05, ref_cloud.points[ii].y, ref_cloud.points[ii].z);
    ref_cloud.points.push_back(p2);
    Point p3(ref_cloud.points[ii].x, ref_cloud.points[ii].y-0.05, ref_cloud.points[ii].z);
    ref_cloud.points.push_back(p3);
    Point p4(ref_cloud.points[ii].x, ref_cloud.points[ii].y+0.05, ref_cloud.points[ii].z);
    ref_cloud.points.push_back(p4);
    Point p5(ref_cloud.points[ii].x, ref_cloud.points[ii].y, ref_cloud.points[ii].z-0.05);
    ref_cloud.points.push_back(p5);
    Point p6(ref_cloud.points[ii].x, ref_cloud.points[ii].y, ref_cloud.points[ii].z+0.05);
    ref_cloud.points.push_back(p6);
  }

  ref_cloud.header.frame_id="/map";
  reference_cloud_ = boost::make_shared<pcl::PointCloud<Point> >(ref_cloud);
  getObjects();
  //getObjectsLimitedFOV(15, "fov_1");
  getObjectsLimitedFOV(direction_, "fov_init");
 // getObjectsLimitedFOV(-15, "fov_3");

  labelObjects();

  initializeVoxelGrid();

  // initialize the planner
  planner_ = Planner(voxel_grid_, false, 1);
  planner_.setFullSamplingPositions(full_sampling_positions_);









  std::string s = boost::lexical_cast<std::string>( (int)input_clouds_.size() );
  topic_name = "arrow_" + s;
  publish_topics_.arrowMsg(topic_name, goal_pose);

  if(cloud_size > 0){
    res.result = res.SUCCEEDED;
    res.goal_pose = goal_pose;
  }else
    res.result = res.FAILED;
  return true;
}

bool ExtractObjects::kinectCloud(nbv_utilities::SendCloud::Request &req,
                                 nbv_utilities::SendCloud::Response &res)
{
  sensor_msgs::PointCloud2 cloud = req.cloud;

  pcl::PointCloud<Point> pcl_cloud;
  pcl::fromROSMsg (cloud, pcl_cloud);
  pcl::PointCloud<Point>::ConstPtr input_cloud_ptr;
  input_cloud_ptr = boost::make_shared<pcl::PointCloud<Point> >(pcl_cloud);

  getTableTop(input_cloud_ptr, true);

  return true;
}

void ExtractObjects::getTableTop(pcl::PointCloud<Point>::ConstPtr &cloud, bool table_top)
{
  pcl::PointCloud<Point>::ConstPtr filtered_cloud_ptr;
  pcl::PointCloud<Point>::ConstPtr cloud_filtered_tmp_1;
  pcl::PointCloud<Point>::ConstPtr cloud_filtered_tmp_2;


  /*
  // for the lowered table
  // cut everything away which is under the table
  cloud_filtered_tmp_1 = utilities_.passthrough_filter(cloud, "z", 0.5, 2.0);
  // cut the side walls
  cloud_filtered_tmp_2 = utilities_.passthrough_filter(cloud_filtered_tmp_1, "y", -1.5, 1.5);
  filtered_cloud_ptr = utilities_.passthrough_filter(cloud_filtered_tmp_2, "x", 0.0, 3.0);
  */

  //for the raised table
  // cut everything away which is under the table
  cloud_filtered_tmp_1 = utilities_.passthrough_filter(cloud, "z", 0.83, 2.5);
  // cut the side walls
  cloud_filtered_tmp_2 = utilities_.passthrough_filter(cloud_filtered_tmp_1, "y", -1.2, 1.2);
  filtered_cloud_ptr = utilities_.passthrough_filter(cloud_filtered_tmp_2, "x", 0.0, 4.0);

  /*
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*filtered_cloud_ptr, msg);
  std::string ss = "fgfgf" ;
  publish_topics_.newTopic(ss, msg);
*/

  pcl::PointCloud<Point>::ConstPtr table_top_ptr;
  //table_top_ptr = utilities_.extractTableTop(filtered_cloud_ptr, 0.015, 250);
  //table_top_ptr = utilities_.extractTableTopNormals(filtered_cloud_ptr, 0.04, 100, 30, 0.1);

  table_top_ptr = utilities_.extractTableTopNormals(filtered_cloud_ptr, 0.035, 100, 100, 0.1);

 // table_top_ptr = utilities_.statistical_outlier_removal(table_top_ptr, 50, 2.5);
  // table_top_ptr = utilities_.statistical_outlier_removal(table_top_ptr, 50, 2.3);




  if(table_top)
  {
    table_tops_.push_back(table_top_ptr);

    //compute the bounding box of the table top
    std::vector<Eigen::Vector3f> table_top_bbx;
    table_top_bbx = utilities_.fitTableTopBbx(table_top_ptr);

    //output the table top size
    float length = 0.0;
    float width = 0.0;
    float height = 0.0;
    float s1 = 1000.0;
    float l1 = 0.0;
    float s2 = 1000.0;
    float l2 = 0.0;
    for(unsigned int i=0; i<4; i++)
    {
      if(table_top_bbx[i].x() > l1)
        l1 = table_top_bbx[i].x();
      if(table_top_bbx[i].x() < s1)
        s1 = table_top_bbx[i].x();
      if(table_top_bbx[i].y() > l2)
        l2 = table_top_bbx[i].y();
      if(table_top_bbx[i].y() < s2)
        s2 = table_top_bbx[i].y();
      if(table_top_bbx[i].z() > height)
        height = table_top_bbx[i].z();
     }
    length = fabs(l1-s1);
    width = fabs(l2-s2);

    ROS_INFO("table top dim - l: %f w: %f h: %f", length, width, height);

    table_top_bbx_.push_back(table_top_bbx);
    publish_topics_.tableTopBbxMsg(table_top_bbx);
  }else
  {
    // get the inverse table top point cloud
    pcl::PointCloud<Point>::ConstPtr inv_table_top_ptr;
    inv_table_top_ptr = utilities_.get_inv_table_top_ptr();
    inv_table_tops_.push_back(inv_table_top_ptr);
  }



  // set the table top bounding box to the size of the table
  /*
  Eigen::Vector3f p1, p2, p3, p4;
  p1 = table_top_bbx[0];
  p2 = table_top_bbx[1];
  p3 = table_top_bbx[2];
  p4 = table_top_bbx[3];

  ROS_INFO("p1: %f %f %f", p1.x(), p1.y(), p1.z() );
  ROS_INFO("p2: %f %f %f", p2.x(), p2.y(), p2.z() );
  ROS_INFO("p3: %f %f %f", p3.x(), p3.y(), p3.z() );
  ROS_INFO("p4: %f %f %f", p4.x(), p4.y(), p4.z() );
  */




}

void ExtractObjects::getObjects()
{
  unsigned int index = inv_table_tops_.size() - 1 ;
  pcl::PointCloud<Point>::ConstPtr objects_ptr;

  // average over the 'z' coordinate of the table top plane
  // get the 'x' nearest front
  double z_coord = 0.0;
  z_coord = utilities_.avg_points_z(table_tops_[index]);

  // filter everything which is below the table plate
  double min_bounds = z_coord + 0.004;
  double max_bounds = 1.6;
  objects_ptr = utilities_.passthrough_filter(inv_table_tops_[index], "z", min_bounds, max_bounds);

  //filter around the table plate
  // deleting artefacts on the table plate boundaries
  Eigen::Vector4f min_table_top, max_table_top;
  pcl::getMinMax3D(*table_tops_[index], min_table_top, max_table_top);
  min_bounds = min_table_top.x() + 0.01;
  max_bounds = max_table_top.x() - 0.01;
  objects_ptr = utilities_.passthrough_filter(objects_ptr, "x", min_bounds, max_bounds);

  min_bounds = min_table_top.y() + 0.01;
  max_bounds = max_table_top.y() - 0.01;
  objects_ptr = utilities_.passthrough_filter(objects_ptr, "y", min_bounds, max_bounds);

  objects_ptr = utilities_.statistical_outlier_removal(objects_ptr, 100, 0.9);

  objects_.push_back(objects_ptr);
}

void ExtractObjects::getObjectsLimitedFOV(float direction, std::string name)
{
  unsigned int index = objects_.size() - 1 ;

  // field of view angle
  float fov = 8.0;
  Eigen::Vector2f p1(table_top_centroid_.x(), table_top_centroid_.y());
  Eigen::Vector2f p2(scan_poses_[scan_poses_.size()-1].x(), scan_poses_[scan_poses_.size()-1].y());

  float theta = direction * M_PI/180;
  Eigen::Matrix2f R;
  R << cos(theta), -sin(theta),
       sin(theta),  cos(theta);

  Eigen::Vector2f p2f(0.0,0.0);
  p2f = R*(p1-p2) + p2;
  p1 = p2f;

  Eigen::Vector2f normal = utilities_.computeNormal(p1,p2,fov);

  pcl::PointCloud<Point> cloud;
  cloud.header = objects_[index]->header;
  cloud.is_dense = false;
  for(unsigned int ii=0; ii<objects_[index]->points.size(); ii++)
  {
    Eigen::Vector2f p2d(objects_[index]->points[ii].x,
                        objects_[index]->points[ii].y);

    Eigen::Vector2f plo = p2d-p2;
    float angle = normal.dot(plo);

    if(angle < 0.0)
    {
      Point point(objects_[index]->points[ii].x,
                  objects_[index]->points[ii].y,
                  objects_[index]->points[ii].z);
      cloud.push_back(point);
    }
  }


  normal = utilities_.computeNormal(p1,p2,-fov);
  pcl::PointCloud<Point> cloud2;
  cloud2.header = cloud.header;
  cloud2.is_dense = false;
  for(unsigned int ii=0; ii<cloud.points.size(); ii++)
  {
    Eigen::Vector2f p2d(cloud.points[ii].x,
                        cloud.points[ii].y);

    Eigen::Vector2f plo = p2d-p2;
    float angle = normal.dot(plo);

    if(angle > 0.0)
    {
      Point point(cloud.points[ii].x, cloud.points[ii].y, cloud.points[ii].z);
      cloud2.push_back(point);
    }
  }


  pcl::PointCloud<Point>::ConstPtr objects_ptr;
  objects_ptr = boost::make_shared<pcl::PointCloud<Point> >(cloud2);
  objects_[index] = objects_ptr;



  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud2, msg);
  std::string ss = name ;
  publish_topics_.newTopic(ss, msg);







//  unsigned int index = inv_table_tops_.size() - 1 ;
//  pcl::PointCloud<Point>::ConstPtr objects_ptr;





//  objects_.push_back(objects_ptr);
}

void ExtractObjects::labelObjects()
{


  seg_objects_.clear();



  pcl::PointCloud<PointRGB> labeled_objects;
  unsigned int index = objects_.size() - 1 ;



  // get for each object a point cloud
  std::vector<pcl::PointCloud<Point>::ConstPtr> objects_ptr_vec;
 // ROS_INFO("TEST-0-3: %d", (int)objects_[index]->points.size());
  if((int)objects_[index]->points.size() > 0)
    objects_ptr_vec = utilities_.extract_euclidiean_cluster(objects_[index], 0.02, 50);

  //ROS_INFO("size: %d", (int)objects_[index]->points.size());



  for(unsigned int obj=0; obj<objects_ptr_vec.size(); obj++)
  {
    pcl::PointCloud<PointRGB> colored_object;
    pcl::PointCloud<Point>::ConstPtr object_ptr;
    object_ptr = objects_ptr_vec[obj];

    colored_object.header = object_ptr->header;
    colored_object.height = object_ptr->height;
    colored_object.width = object_ptr->width;
    colored_object.is_dense = object_ptr->is_dense;
    colored_object.points.resize(object_ptr->points.size());

    for(int ii=0; ii < (int)colored_object.points.size(); ii++)
    {
      colored_object.points[ii].x = object_ptr->points[ii].x;
      colored_object.points[ii].y = object_ptr->points[ii].y;
      colored_object.points[ii].z = object_ptr->points[ii].z;
      colored_object.points[ii].rgb = colors_[obj];
    }

    if(obj<1)
      labeled_objects = colored_object;
    else
      labeled_objects += colored_object;

    seg_objects_.push_back(colored_object);
  }



  pcl::PointCloud<PointRGB>::ConstPtr labeled_objects_ptr;
  labeled_objects_ptr = boost::make_shared<pcl::PointCloud<PointRGB> >(labeled_objects);
  labeled_objects_.push_back(labeled_objects_ptr);



  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(labeled_objects, msg);
  // create a new publishing object
  std::string s = boost::lexical_cast<std::string>( (int)input_clouds_.size()-1 );
  std::string ss = "scan_" + s + "/labeled_objects";
  publish_topics_.newTopic(ss, msg);



  if(seg_objects_.size() > 0)
  {
    pcl::toROSMsg(seg_objects_[0], msg);
    std::string tt = "test";
    publish_topics_.newTopic(tt, msg);
  }



}

std::vector<Eigen::Vector3f>
  ExtractObjects::samplingPositions2(unsigned int n, double laser_tilt_mount_z, Eigen::Vector4f& centroid, float radius)
{
  std::vector<Eigen::Vector3f> sampling_positions;

  // degree
  int deg = 360 / (n);

  for(unsigned int ii=0; ii<n; ii++)
  {
    Eigen::Vector3f pos;
    pos.x() = centroid.x() - radius * cos((deg*ii)*M_PI/180);
    pos.y() = centroid.y() + radius * sin((deg*ii)*M_PI/180);
    pos.z() = laser_tilt_mount_z;

    sampling_positions.push_back(pos);
  }

  return sampling_positions;
}

void ExtractObjects::samplingPositions(unsigned int n, double laser_tilt_mount_z, Eigen::Vector4f& centroid, float radius)
{
  // sampling angles
  std::vector<float> sa;
  sa.push_back(15.0);
  sa.push_back(0.0);
  sa.push_back(-15.0);

  // degree
  int deg = 360 / (n);

  for(unsigned int ii=0; ii<n; ii++)
  {
    Eigen::Vector3f pos;
    pos.x() = centroid.x() - radius * cos((deg*ii)*M_PI/180);
    pos.y() = centroid.y() + radius * sin((deg*ii)*M_PI/180);
    pos.z() = laser_tilt_mount_z;

    sampling_positions_.push_back(pos);
    sampling_angle_.push_back(sa);
  }

  //publish sampling positions as visualization markers
  publish_topics_.samplingPositionsMsg(sampling_positions_);
}

void ExtractObjects::initializeVoxelGrid()
{

  int index=0;
 // if(init_)
 //   index = 1;

  // compute the dimensions of the table top bounding box
  Eigen::Vector4f min_objects(0,0,0,0), max_objects(0,0,0,0);
  if(labeled_objects_[0]->points.size() > 0)
    pcl::getMinMax3D(*labeled_objects_[0], min_objects, max_objects);

  if(max_objects.z()-min_objects.z() < 0.12)
    max_objects.z() = min_objects.z() + 0.12;

  Eigen::Vector3f p1, p2, p3;
  p1 = table_top_bbx_[index][0];
  p2 = table_top_bbx_[index][1];
  p3 = table_top_bbx_[index][3];
  float depth = sqrt( pow(p2.x() - p1.x(),2) +  pow(p2.y() - p1.y(),2) +  pow(p2.z() - p1.z(),2) );
  float width = sqrt( pow(p3.x() - p1.x(),2) +  pow(p3.y() - p1.y(),2) +  pow(p3.z() - p1.z(),2) );
  //float height = sqrt( pow(table_top_centroid_.z() - (max_objects.z() + 0.15),2) )  ;
  float height = sqrt( pow(min_objects.z() - (max_objects.z() + 0.01),2) )  ;

  //ROS_INFO("depth: %f", depth);
  //ROS_INFO("width: %f", width);

  Eigen::Vector4f voxel_grid_centroid_;
  voxel_grid_centroid_ = table_top_centroid_;
  voxel_grid_centroid_.z() = min_objects.z() + (height/2);

  publish_topics_.centroidPositionsMsg(voxel_grid_centroid_);

  Eigen::Vector3f voxel_grid_bbx(depth, width, height);
  Eigen::Vector3i dim(129, 129, 65);
  voxel_grid_ = new VoxelGrid(voxel_grid_bbx, dim,
                                        voxel_grid_centroid_, table_top_bbx_[index],
                                        publish_topics_);
  voxel_grid_->setSamplingPositions(sampling_positions_);

  // calculate the angle in which the table is rotated
  Eigen::Vector3f vec = p1-p3;
  vec.normalize();
  Eigen::Vector3f ax(0.0,-1.0,0.0);
  float dot = vec.dot(ax);
  float angle = acos(dot);
  angle =  (M_PI/2 - angle) +  M_PI/2;
  //angle = (180.0*3.14/180)-angle;
  voxel_grid_->setRotAngle(angle);

  ROS_INFO("rot angle: %f", angle*180/3.14);

  /*
  Eigen::Vector3f n(0.0, 1.0, 0.0);
  float theta = angle;
  float xr = n.x()*cos(theta)-n.y()*sin(theta);
  float yr = n.x()*sin(theta)+n.y()*cos(theta);
  n.x() = xr;
  n.y() = yr;
  n.normalize();

  Eigen::Vector3f p_0(0.0,0.0,0.0);
  Eigen::Vector3f p_1(n.x(),n.y(),0.0);

  publish_topics_.robotPositionsMsg(p_0);
  publish_topics_.robotPositionsMsg(p_1);
*/

  ROS_INFO("ExtractObjects - initializeVoxelGrid: DONE");

}

void ExtractObjects::setNewVoxelGridScan(pcl::PointCloud<PointRGB>::ConstPtr& cloud)
{

  float angle = voxel_grid_->getRotAngle();
  voxel_grid_->setGridPoints(cloud);
  ROS_INFO("ExtractObjects - setGridPoints: Voxel Grid - DONE");
  voxel_grid_->voxelGridError();

  // get the dimensions of the objects
  // messy stuff here:
  // first rotate the point cloud the angel in which the table is rotated,
  // this is because to compute the min/max values of the point cloud which
  // bound the cloud, the cloud has to be axis aligned.
  // Then compute the min/max values , rotate the points back to the original
  // point cloud
  Eigen::Vector4f min_object, max_object;
  std::vector<Eigen::VectorXf> objd;
  for(int i=0; i < (int)seg_objects_.size(); i++)
  {
    pcl::PointCloud<PointRGB> tmp_cloud;
    geometry_msgs::Quaternion tmp = tf::createQuaternionMsgFromYaw(-angle);
    Eigen::Quaternionf rot(tmp.w, tmp.x, tmp.y, tmp.z);
    Eigen::Vector3f offset(0.0,0.0,0.0);
    pcl::transformPointCloud(seg_objects_[i], tmp_cloud, offset, rot);

    pcl::getMinMax3D(tmp_cloud, min_object, max_object);

    float theta = angle;
    float xr = min_object.x()*cos(theta)-min_object.y()*sin(theta);
    float yr = min_object.x()*sin(theta)+min_object.y()*cos(theta);
    min_object.x() = xr;
    min_object.y()= yr;

    xr = max_object.x()*cos(theta)-max_object.y()*sin(theta);
    yr = max_object.x()*sin(theta)+max_object.y()*cos(theta);
    max_object.x() = xr;
    max_object.y()= yr;

    Eigen::VectorXf obj(6);
    obj(0) = min_object(0);
    obj(1) = min_object(1);
    obj(2) = min_object(2);
    obj(3) = max_object(0);
    obj(4) = max_object(1);
    obj(5) = max_object(2);

    objd.push_back(obj);
  }
  voxel_grid_->setObjectDimensions(objd);

  /*
  Eigen::Vector3f n(0.0, 1.0, 0.0);
  float theta = angle;
  float xr = n.x()*cos(theta)-n.y()*sin(theta);
  float yr = n.x()*sin(theta)+n.y()*cos(theta);
  n.x() = xr;
  n.y() = yr;
  n.normalize();

  Eigen::Vector3f z(0.0,0.0,0.0);
  publish_topics_.robotPositionsMsg(z);
  publish_topics_.robotPositionsMsg(n);
*/


}

void ExtractObjects::transformClouds(int& index)
{
  ROS_INFO("Transform cloud");

  // set the point cloud
  pcl::PointCloud<Point> cloud;
  cloud.header.frame_id = "/map";
  for(unsigned int ii=0; ii<4; ii++)
  {
    Point point(table_top_bbx_[index][ii].x(),
                table_top_bbx_[index][ii].y(),
                table_top_bbx_[index][ii].z());
    cloud.points.push_back(point);
  }
  // add more points to the cloud
  for(unsigned int ii=0; ii<4; ii++)
  {
    Point p(cloud.points[ii].x-0.05, cloud.points[ii].y, cloud.points[ii].z);
    cloud.points.push_back(p);
    Point p2(cloud.points[ii].x+0.05, cloud.points[ii].y, cloud.points[ii].z);
    cloud.points.push_back(p2);
    Point p3(cloud.points[ii].x, cloud.points[ii].y-0.05, cloud.points[ii].z);
    cloud.points.push_back(p3);
    Point p4(cloud.points[ii].x, cloud.points[ii].y+0.05, cloud.points[ii].z);
    cloud.points.push_back(p4);
    Point p5(cloud.points[ii].x, cloud.points[ii].y, cloud.points[ii].z-0.05);
    cloud.points.push_back(p5);
    Point p6(cloud.points[ii].x, cloud.points[ii].y, cloud.points[ii].z+0.05);
    cloud.points.push_back(p6);
  }

  pcl::PointCloud<Point>::ConstPtr cloud_ptr;
  cloud_ptr = boost::make_shared<pcl::PointCloud<Point> >(cloud);

  pcl::PointCloud<Point> input_cloud = *input_clouds_[index];
  utilities_.transformCloud(reference_cloud_, cloud_ptr, input_cloud);
  input_clouds_[index] = boost::make_shared<pcl::PointCloud<Point> >(input_cloud);

  pcl::PointCloud<Point> table_top = *table_tops_[index];
  utilities_.transformCloud(reference_cloud_, cloud_ptr, table_top);
  table_tops_[index] = boost::make_shared<pcl::PointCloud<Point> >(table_top);

  pcl::PointCloud<Point> inv_table_top = *inv_table_tops_[index];
  utilities_.transformCloud(reference_cloud_, cloud_ptr, inv_table_top);
  inv_table_tops_[index] = boost::make_shared<pcl::PointCloud<Point> >(inv_table_top);

  //index = index - 1;
  //pcl::PointCloud<Point> objects = *objects_[index];
  //utilities_.transformCloud(reference_cloud_, cloud_ptr, objects);
  //objects_[index] = boost::make_shared<pcl::PointCloud<Point> >(objects);

  //pcl::PointCloud<PointRGB> labeled_objects = *labeled_objects_[index];

  //labeled_objects_[index] = boost::make_shared<pcl::PointCloud<Point> >(labeled_objects);

}

void ExtractObjects::visualize()
{
  int index;

  // publish input point cloud
  index = input_clouds_.size() - 1;
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*input_clouds_[index], msg);
  std::string s = boost::lexical_cast<std::string>( (int)input_clouds_.size()-1 );
  std::string topic_name = "scan_" + s + "/input_cloud";
  publish_topics_.newTopic(topic_name, msg);

  // publish the table top point cloud
  index = table_tops_.size() - 1;
  topic_name = "scan_" + s;
  pcl::toROSMsg(*table_tops_[index], msg);
  std::string ss = topic_name + "/table_top";
  publish_topics_.newTopic(ss, msg);

  // publish the inverse of the table top point cloud
  index = inv_table_tops_.size() - 1;
  pcl::toROSMsg(*inv_table_tops_[index], msg);
  ss = topic_name + "/inv_table_top";
  publish_topics_.newTopic(ss, msg);

  // pubslish the objects
  index = objects_.size() - 1;
  pcl::toROSMsg(*objects_[index], msg);
  ss = "scan_" + s + "/objects";
  publish_topics_.newTopic(ss, msg);

}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "extractObjects");
  ros::NodeHandle n;

  ExtractObjects extractObjects(n);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
