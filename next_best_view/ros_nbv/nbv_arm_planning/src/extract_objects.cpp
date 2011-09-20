/*
 * extract_objects.cpp
 *
 *  Created on: Dec 2, 2010
 *      Author: Christian Potthast
 */

#include <nbv_arm_planning/extract_objects.h>


ExtractObjects::ExtractObjects(ros::NodeHandle& n)
{
  node_handle_ = n;
  utilities_ = Utilities();
  publish_topics_ = PublishTopics(node_handle_, "/base_link");

  // Service
  send_cloud_srv_ = node_handle_.advertiseService("extractObjects/SendCloud", &ExtractObjects::sendCloud, this);
  find_table_srv_ = node_handle_.advertiseService("extractObjects/FindTable", &ExtractObjects::findTable, this);
  get_arm_pose_goal_srv_ = node_handle_.advertiseService("extractObjects/GetArmPoseGoal", &ExtractObjects::getArmPoseGoal, this);

  // Start the timer that will trigger the processing loop (timerCallback)
  timer_ = node_handle_.createTimer(ros::Duration(2,0), &ExtractObjects::timerCallback, this);

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

  sppose_ = 0;

}

void ExtractObjects::timerCallback(const ros::TimerEvent& event)
{
  publish_topics_.Publish();
}


bool ExtractObjects::sendCloud(nbv_utilities::SendCloud::Request &req,
                               nbv_utilities::SendCloud::Response &res)
{
  /*

  sensor_msgs::PointCloud2 cloud = req.cloud;
  int cloud_size = cloud.width * cloud.height;
  geometry_msgs::PoseWithCovarianceStamped robot_pose = req.robot_pose;
  Eigen::Vector3f scan_pos;
  scan_pos.x() =  robot_pose.pose.pose.position.x;
  scan_pos.y() =  robot_pose.pose.pose.position.y;
  scan_pos.z() =  robot_pose.pose.pose.position.z;
  scan_poses_.push_back(scan_pos);
  publish_topics_.robotPositionsMsg(scan_pos);

  ROS_INFO("ExtractObjects - sendCloud: size of the cloud: %d", cloud_size );

  // convert the point cloud2 msg into the pcl point cloud format
  pcl::PointCloud<Point> pcl_cloud;
  pcl::fromROSMsg (cloud, pcl_cloud);
  pcl::PointCloud<Point>::ConstPtr input_cloud_ptr;
  input_cloud_ptr = boost::make_shared<pcl::PointCloud<Point> >(pcl_cloud);
  input_clouds_.push_back(input_cloud_ptr);

  getTableTop(input_cloud_ptr);

  int index = input_clouds_.size() - 1 ;
  unsigned int init_case = 0;
  if(init_ == false)
    init_case = 1;
  else
    init_case = 2;

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

    initializeOccupnacyGrid();

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
  bool state = planner_.plan_expectation(P0, sampling_positions_);
  //bool state = planner_.plan_simple(P0, sampling_positions_);
  //bool state = planner_.collect_scans(P0, sampling_positions_);
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
*/
  /*
    }else
      res.result = res.DONE;
  }else
    res.result = res.FAILED;
*/


  ROS_INFO("goal pose: %f %f ", res.goal_pose.pose.position.x, res.goal_pose.pose.position.y);

  return true;
}


bool ExtractObjects::getArmPoseGoal(nbv_utilities::SendCloud::Request &req,
                                    nbv_utilities::SendCloud::Response &res)
{
  sensor_msgs::PointCloud2 cloud = req.cloud;
  int cloud_size = cloud.width * cloud.height;
  geometry_msgs::PoseWithCovarianceStamped robot_pose = req.robot_pose;
  Eigen::Vector3f scan_pos;
  scan_pos.x() =  robot_pose.pose.pose.position.x;
  scan_pos.y() =  robot_pose.pose.pose.position.y;
  scan_pos.z() =  robot_pose.pose.pose.position.z;
  scan_poses_.push_back(scan_pos);
  publish_topics_.robotPositionsMsg(scan_pos);

  ROS_INFO("ExtractObjects - sendCloud: size of the cloud: %d", cloud_size );

  // convert the point cloud2 msg into the pcl point cloud format
  pcl::PointCloud<Point> pcl_cloud;
  pcl::fromROSMsg (cloud, pcl_cloud);
  pcl::PointCloud<Point>::ConstPtr input_cloud_ptr;
  input_cloud_ptr = boost::make_shared<pcl::PointCloud<Point> >(pcl_cloud);
  input_clouds_.push_back(input_cloud_ptr);

  getTableTop(input_cloud_ptr);

  int index = input_clouds_.size() - 1 ;
  unsigned int init_case = 0;
  if(init_ == false)
    init_case = 1;
  //else
   // init_case = 2;

  //ROS_INFO("init_case: %d ; size: %d", init_case, (int)input_clouds_.size());

  //Eigen::Vector3f new_scan_position(0.0,0.0,0.0);
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

    /*
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
    */


    getObjects();
    //getObjectsLimitedFOV(15, "fov_1");
   // getObjectsLimitedFOV(direction_, "fov_init");
   // getObjectsLimitedFOV(-15, "fov_3");

    labelObjects();

    initializeOccupnacyGrid();

    setNewVoxelGridScan(labeled_objects_[labeled_objects_.size()-1]);
    // initialize the planner
    planner_ = Planner(occupancy_grid_, false, 1);
    //planner_.setFullSamplingPositions(full_sampling_positions_);


  }else{
    //transformClouds(index);
    getObjects();
    //getObjectsLimitedFOV(direction_, "fov");
    labelObjects();

    setNewVoxelGridScan(labeled_objects_[labeled_objects_.size()-1]);
    //ROS_INFO("Transform cloud - DONE");
  }


  // call the sampling poses service
  ros::service::waitForService("nbv_arm_planning/PoseSamplingNode");
  geometry_msgs::Vector3 center;
  center.x = table_top_centroid_.x();
  center.y = table_top_centroid_.y();
  center.z = table_top_centroid_.z();

  nbv_arm_planning::SamplingPosesRequest sampling_poses_req;
  nbv_arm_planning::SamplingPosesResponse sampling_poses_res;

  sampling_poses_req.req = true;
  sampling_poses_req.center = center;

  if (!ros::service::call("nbv_arm_planning/PoseSamplingNode", sampling_poses_req, sampling_poses_res)){
    ROS_ERROR("ExtractObjects : could not receive sampled poses");
    return false;
  }else{
    ROS_INFO("ExtractObjects : received sampled poses");
  }

//  sampling_poses_ = sampling_poses_res.poses;
    sampling_poses_ = sampling(sppose_);

    geometry_msgs::PoseArray sampling_poses_1 = sampling_poses_res.poses;






/*
  for(int i=0; i<sampling_poses_.poses.size(); i++)
  {
    std::string s = boost::lexical_cast<std::string>( i );
    std::string topic_name = "sp_" + s;
    publish_topics_.arrowMsg(topic_name, sampling_poses_.poses[i]);

  }
*/


  const Eigen::Vector3f P0 = scan_poses_[scan_poses_.size()-1];

  std::string s = boost::lexical_cast<std::string>( (int)input_clouds_.size() );
  std::string topic_name = "arrow_" + s;
  publish_topics_.arrowMsg(topic_name, robot_pose.pose.pose);

  tf::Quaternion quat;
  quat.setX(robot_pose.pose.pose.orientation.x);
  quat.setY(robot_pose.pose.pose.orientation.y);
  quat.setZ(robot_pose.pose.pose.orientation.z);
  quat.setW(robot_pose.pose.pose.orientation.w);

  ROS_INFO("tf: %f %f %f %f", robot_pose.pose.pose.orientation.x, robot_pose.pose.pose.orientation.y, robot_pose.pose.pose.orientation.z, robot_pose.pose.pose.orientation.w);

  btMatrix3x3 rot(quat);

  btVector3 t(1,0,0);
  btVector3 r = rot * t;

 // Eigen::Vector3f view_dir(robot_pose.pose.pose.orientation.x, robot_pose.pose.pose.orientation.y, robot_pose.pose.pose.orientation.z);

  Eigen::Vector3f test;
 // Eigen::Vector3f b;
 // b.UnitX();


 // test.x() = view_dir.x()*b.x();
 // test.y() = view_dir.y()*b.y();
 // test.z() = view_dir.z()*b.z();

  Eigen::Vector4f c;
  pcl::compute3DCentroid(*input_cloud_ptr, c);

  test.x() = r.x() + robot_pose.pose.pose.position.x;
  test.y() = r.y() + robot_pose.pose.pose.position.y;
  test.z() = r.z() + robot_pose.pose.pose.position.z;

  Eigen::Vector3f test2;
  test2.x() = P0.x()+0.3;
  test2.y() = P0.y();
  test2.z() = P0.z();


  //publish_topics_.robotPositionsMsg(test);


  occupancy_grid_->setViewDir(test);
  bool state = planner_.plan_expectation_kinect(P0, sampling_poses_);

  geometry_msgs::Pose g_pose;
  g_pose.position.x = 0.0;
  g_pose.position.y = 0.0;
  g_pose.position.z = 0.0;

  geometry_msgs::Pose new_scan_position;
  new_scan_position = planner_.getNewScanPosition();

  topic_name = "nbv_arrow";
  publish_topics_.arrowMsg(topic_name, new_scan_position);


  ///////////////////////////////////////////////
  // GRAPH
  //////////////////////////////////////////////
/*
  Graph graph(robot_pose.pose.pose);
  for(unsigned int ii=0; ii<sampling_poses_.poses.size(); ii++)
  {
    graph.insertNewNode(sampling_poses_.poses[ii]);
  }
  graph.computeGoalDist(new_scan_position);
  std::vector<geometry_msgs::PoseArray> paths = graph.getPaths(new_scan_position);
  std::vector<geometry_msgs::PoseArray> goal_paths = graph.getGoalPaths(paths, new_scan_position);

  ROS_INFO("paths: %d", (int)goal_paths.size());
  for( int i=0; i<(int)goal_paths.size(); i++)
  {
    std::string s = boost::lexical_cast<std::string>( i );
    std::string topic_name = "path_" + s;
    publish_topics_.PathMsg(topic_name, goal_paths[i]);
  }

  //graph.print(goal_paths);

  publish_topics_.sampledPosesMsg(sampling_poses_);

  s = boost::lexical_cast<std::string>( (int)input_clouds_.size() );
  topic_name = "arrow_" + s;
  publish_topics_.arrowMsg(topic_name, new_scan_position);

*/

/*
  geometry_msgs::Pose pose_test;
  pose_test.position.x = 0.0;
  pose_test.position.y = 0.0;
  pose_test.position.z = 0.0;
  Graph graph_test(pose_test);
  graph_test.test();

  Graph::node* root = graph_test.getRoot();
//  graph_test.print(root);

  Graph::node* node_1 = root->edge[0];
  Graph::node* node_2 = root->edge[1];
  Graph::node* node_3 = root->edge[2];
  Graph::node* node_4 = root->edge[3];
 // graph_test.print(node_1);
 // graph_test.print(node_2);
 // graph_test.print(node_3);
 // graph_test.print(node_4);

  geometry_msgs::Pose g_pose;
  g_pose.position.x = 0.0;
  g_pose.position.y = 0.8;
  g_pose.position.z = 0.0;

  graph_test.computeGoalDist(g_pose);
  graph_test.print(root);
   graph_test.print(node_1);
   graph_test.print(node_2);
   graph_test.print(node_3);
   graph_test.print(node_4);
  std::vector<geometry_msgs::PoseArray> paths = graph_test.getPaths(g_pose);
  std::vector<geometry_msgs::PoseArray> goal_paths = graph_test.getGoalPaths(paths, g_pose);
  graph_test.print(goal_paths);
*/

  /*
  const Eigen::Vector3f P0 = scan_poses_[scan_poses_.size()-1];
  //planner_.plan(P0, sampling_positions_);
  //bool state = planner_.plan_greedy(P0, sampling_positions_);
  //bool state = planner_.plan_expectation(P0, sampling_positions_);
  //bool state = planner_.plan_simple(P0, sampling_positions_);
  //bool state = planner_.collect_scans(P0, sampling_positions_);
  //bool state = planner_.plan_simple_FOV(P0, full_sampling_positions_, sampling_angle_, direction_);
  bool state = planner_.plan_expectation_FOV(P0, full_sampling_positions_, sampling_angle_, direction_);


ros
  new_scan_position = planner_.getNewScanPosition();
  direction_ = planner_.getNewDirection();
  //ROS_INFO("%f %f %f", new_scan_position.x(), new_scan_position.y(), new_scan_position.z());
  publish_topics_.robotPositionsMsg(new_scan_position);

  //publish_topics_.robotPositionsMsg(sampling_positions_[5]);

*/



  visualize();
  ROS_INFO("Visualize cloud - DONE");











/*
  if(cloud_size > 0){
    double theta = atan2(table_top_centroid_.y() - new_scan_position.y(),
                           table_top_centroid_.x() - new_scan_position.x());

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    geometry_msgs::PoseStamped goal_pose;

    goal_pose.pose.position.x = table_top_centroid_.x() - 0.1;
    goal_pose.pose.position.y = table_top_centroid_.y();
    goal_pose.pose.position.z = table_top_centroid_.z() + 0.05;

    //goal_pose.pose.position.x = new_scan_position.x();
    //goal_pose.pose.position.y = new_scan_position.y();
    //goal_pose.pose.position.z = new_scan_position.z();
    goal_pose.pose.orientation = odom_quat;


    std::string s = boost::lexical_cast<std::string>( (int)input_clouds_.size() );
    std::string topic_name = "arrow_" + s;
    publish_topics_.arrowMsg(topic_name, goal_pose);



    res.result = res.SUCCEEDED;
    res.goal_pose.pose = goal_pose.pose;
  }
*/

 // if(input_clouds_.size() == 2 || state == false)
    res.goal_pose.pose = new_scan_position;
    res.result = res.DONE;

  /*
    }else
      res.result = res.DONE;
  }else
    res.result = res.FAILED;
*/


 // ROS_INFO("goal pose: %f %f %f", res.goal_pose.pose.position.x, res.goal_pose.pose.position.y, res.goal_pose.pose.position.z);
 // ROS_INFO("goal orientation: %f %f %f %f", res.goal_pose.pose.orientation.x, res.goal_pose.pose.orientation.y, res.goal_pose.pose.orientation.z, res.goal_pose.pose.orientation.w);



    printf("geometry_msgs::PoseArray sampling_poses;\n");
    printf("geometry_msgs::Pose t;\n");
    for(unsigned int i=0 ; i<sampling_poses_.poses.size(); i++)
    {
      printf("t.position.x = %f; \n", sampling_poses_1.poses[i].position.x);
      printf("t.position.y = %f; \n", sampling_poses_1.poses[i].position.y);
      printf("t.position.z = %f;\n", sampling_poses_1.poses[i].position.z);
      printf("t.orientation.x = %f;\n", sampling_poses_1.poses[i].orientation.x);
      printf("t.orientation.y = %f;\n", sampling_poses_1.poses[i].orientation.y);
      printf("t.orientation.z = %f;\n", sampling_poses_1.poses[i].orientation.z);
      printf("t.orientation.w = %f;\n", sampling_poses_1.poses[i].orientation.w);
      printf("sampling_poses.poses.push_back(t);\n");
    }

    std::cout << "--------------" << std::endl;

    std::cout << "pose.pose.position.x = " << res.goal_pose.pose.position.x << ";" << std::endl;
    std::cout << "pose.pose.position.y = " << res.goal_pose.pose.position.y << ";" << std::endl;
    std::cout << "pose.pose.position.z = " << res.goal_pose.pose.position.z << ";" << std::endl;
    std::cout << "pose.pose.orientation.x = " << res.goal_pose.pose.orientation.x << ";" << std::endl;
    std::cout << "pose.pose.orientation.y = " << res.goal_pose.pose.orientation.y << ";" << std::endl;
    std::cout << "pose.pose.orientation.z = " << res.goal_pose.pose.orientation.z << ";" << std::endl;
    std::cout << "pose.pose.orientation.w = " << res.goal_pose.pose.orientation.w << ";" << std::endl;


    sppose_++;

  return true;
}



bool ExtractObjects::findTable(nbv_utilities::FindTable::Request &req,
                               nbv_utilities::FindTable::Response &res)
{
  init_ = true;

  sensor_msgs::PointCloud2 cloud = req.cloud;
  int cloud_size = cloud.width * cloud.height;
  geometry_msgs::PoseWithCovarianceStamped robot_pose = req.robot_pose;
  Eigen::Vector3f scan_pos;
  scan_pos.x() =  robot_pose.pose.pose.position.x;
  scan_pos.y() =  robot_pose.pose.pose.position.y;
  scan_pos.z() =  robot_pose.pose.pose.position.z;
  scan_poses_.push_back(scan_pos);
  //publish_topics_.robotPositionsMsg(scan_pos);

  ROS_INFO("ExtractObjects - findTable: size of the cloud: %d", cloud_size );

  // convert the point cloud2 msg into the pcl point cloud format
  pcl::PointCloud<Point> pcl_cloud;
  pcl::fromROSMsg (cloud, pcl_cloud);
  pcl::PointCloud<Point>::ConstPtr input_cloud_ptr;
  input_cloud_ptr = boost::make_shared<pcl::PointCloud<Point> >(pcl_cloud);
  input_clouds_.push_back(input_cloud_ptr);

  // create a new publishing object
  std::string topic_name = "init_scan/input_cloud";
  publish_topics_.newTopic(topic_name, cloud);

  getTableTop(input_cloud_ptr);

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

  // get the centroid of the bounding box
  table_top_centroid_.x() = table_top_bbx_[index][4].x();
  table_top_centroid_.y() = table_top_bbx_[index][4].y();
  table_top_centroid_.z() = table_top_bbx_[index][4].z();

  getObjects();

  labelObjects();

  initializeOccupnacyGrid();
  planner_ = Planner(occupancy_grid_, false, 1);

  /*
  if(cloud_size > 0){
    res.result = res.SUCCEEDED;
    res.goal_pose = goal_pose;
  }else
    res.result = res.FAILED;
    */
  return true;
}

void ExtractObjects::getTableTop(pcl::PointCloud<Point>::ConstPtr &cloud)
{
  //pcl::PointCloud<Point>::ConstPtr filtered_cloud_ptr;
  pcl::PointCloud<Point>::ConstPtr cloud_filtered_tmp_1;
  //pcl::PointCloud<Point>::ConstPtr cloud_filtered_tmp_2;


  // for the lowered table
  // cut everything away which is under the table
  cloud_filtered_tmp_1 = utilities_.passthrough_filter(cloud, "z", 0.4, 2.0);
  // cut the side walls
  /*
  cloud_filtered_tmp_2 = utilities_.passthrough_filter(cloud_filtered_tmp_1, "y", -1.5, 1.5);
  filtered_cloud_ptr = utilities_.passthrough_filter(cloud_filtered_tmp_2, "x", 0.0, 3.0);
  */

  //for the raised table
  // cut everything away which is under the table
  //cloud_filtered_tmp_1 = utilities_.passthrough_filter(cloud, "z", 0.7, 2.0);
  // cut the side walls
  //cloud_filtered_tmp_2 = utilities_.passthrough_filter(cloud_filtered_tmp_1, "y", -1.2, 1.2);
  //filtered_cloud_ptr = utilities_.passthrough_filter(cloud_filtered_tmp_2, "x", 0.0, 4.0);

  //sensor_msgs::PointCloud2 msg;
  //pcl::toROSMsg(*cloud, msg);
  //std::string ss = "fgfgf" ;
  //publish_topics_.newTopic(ss, msg);
  pcl::PointCloud<Point>::ConstPtr table_top_ptr;
  //table_top_ptr = utilities_.extractTableTop(cloud, 0.015, 250);

  //table_top_ptr = utilities_.extractTableTopNormals(cloud, 0.04, 100, 30, 0.1);
  table_top_ptr = utilities_.extractTableTopNormals(cloud_filtered_tmp_1, 0.035, 100, 100, 0.1);

  table_top_ptr = utilities_.statistical_outlier_removal(table_top_ptr, 50, 2.5);
  table_tops_.push_back(table_top_ptr);

  // get the inverse table top point cloud
  pcl::PointCloud<Point>::ConstPtr inv_table_top_ptr;
  inv_table_top_ptr = utilities_.get_inv_table_top_ptr();
  inv_table_tops_.push_back(inv_table_top_ptr);

  //compute the bounding box of the table top
  std::vector<Eigen::Vector3f> table_top_bbx;
  table_top_bbx = utilities_.fitTableTopBbx(table_top_ptr);

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

  table_top_bbx_.push_back(table_top_bbx);
  publish_topics_.tableTopBbxMsg(table_top_bbx);


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
  /*
  Eigen::Vector4f min_table_top, max_table_top;
  pcl::getMinMax3D(*table_tops_[index], min_table_top, max_table_top);
  min_bounds = min_table_top.x() + 0.01;
  max_bounds = max_table_top.x() - 0.01;
  objects_ptr = utilities_.passthrough_filter(objects_ptr, "x", min_bounds, max_bounds);

  min_bounds = min_table_top.y() + 0.01;
  max_bounds = max_table_top.y() - 0.01;
  objects_ptr = utilities_.passthrough_filter(objects_ptr, "y", min_bounds, max_bounds);
*/
  objects_ptr = utilities_.statistical_outlier_removal(objects_ptr, 100, 0.9);

  objects_.push_back(objects_ptr);
}

void ExtractObjects::getObjectsLimitedFOV(float direction, std::string name)
{
  unsigned int index = objects_.size() - 1 ;

  // field of view angle
  float fov = 7.5;
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
  //ROS_INFO("TEST-0-3: %d", (int)objects_[index]->points.size());
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

  for(unsigned int ii=0; ii<seg_objects_.size(); ii++)
  {
    pcl::toROSMsg(seg_objects_[ii], msg);
    // create a new publishing object
    std::string s = boost::lexical_cast<std::string>( (int)input_clouds_.size()-1 );
    std::string s2 = boost::lexical_cast<std::string>( (int)ii );
    std::string ss = "scan_" + s + "/segmented_objects/cloud_" + s2;
    publish_topics_.newTopic(ss, msg);

  }


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

void ExtractObjects::initializeOccupnacyGrid()
{

  int index=0;
  //if(init_)
   // index = 1;

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
  float height = sqrt( pow(min_objects.z() - (max_objects.z() + 0.01),2) )  ;

  Eigen::Vector4f occupnacy_grid_centroid;
  occupnacy_grid_centroid = table_top_centroid_;
  occupnacy_grid_centroid.z() = min_objects.z() + (height/2);

  publish_topics_.centroidPositionsMsg(occupnacy_grid_centroid);

  Eigen::Vector3f occupancy_grid_bbx(depth, width, height);
  Eigen::Vector3i dim(129, 129, 65);
 // Eigen::Vector3i dim(16, 16, 16);
  occupancy_grid_ = new OccupancyGrid(occupancy_grid_bbx, dim,
                                      occupnacy_grid_centroid, table_top_bbx_[index],
                                        publish_topics_);

  // calculate the angle in which the table is rotated
  Eigen::Vector3f vec = p1-p3;
  vec.normalize();
  Eigen::Vector3f ax(0.0,-1.0,0.0);
  float dot = vec.dot(ax);
  float angle = acos(dot);
  angle = (180.0*3.14/180)-angle;
  occupancy_grid_->setRotAngle(angle);

  ROS_INFO("rot angle: %f", angle*180/3.14);
  ROS_INFO("Occupancy grid centroid: %f %f %f", occupnacy_grid_centroid.x(),
                                                occupnacy_grid_centroid.y(),
                                                occupnacy_grid_centroid.z());


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

  ROS_INFO("ExtractObjects - initializeOccupancyGrid: DONE");

}

void ExtractObjects::setNewVoxelGridScan(pcl::PointCloud<PointRGB>::ConstPtr& cloud)
{
  //float angle = occupancy_grid_->getRotAngle();


  occupancy_grid_->setMeasurements(cloud);

  //occupancy_grid_->bayesianUpdate();
  occupancy_grid_->measurementsUpdate();

  occupancy_grid_->setStates();




/*
  float angle = voxel_grid_->getRotAngle();
  voxel_grid_->setGridPoints(cloud);
  ROS_INFO("ExtractObjects - setGridPoints: Voxel Grid - DONE");
  voxel_grid_->voxelGridError();
*/

  float angle = occupancy_grid_->getRotAngle();
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
  occupancy_grid_->setObjectDimensions(objd);


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

geometry_msgs::PoseArray ExtractObjects::sampling(int i)
{
  std::vector<geometry_msgs::PoseArray> sposes;

  geometry_msgs::PoseArray sampling_poses;
  geometry_msgs::Pose t;
  t.position.x = 0.405916;
  t.position.y = -0.571889;
  t.position.z = 1.284132;
  t.orientation.x = 0.371175;
  t.orientation.y = 0.379470;
  t.orientation.z = 0.169630;
  t.orientation.w = 0.830335;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.269949;
  t.position.y = -0.076637;
  t.position.z = 1.312817;
  t.orientation.x = -0.347973;
  t.orientation.y = 0.425672;
  t.orientation.z = -0.181678;
  t.orientation.w = 0.815299;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.502537;
  t.position.y = 0.236078;
  t.position.z = 1.290139;
  t.orientation.x = -0.101585;
  t.orientation.y = 0.518206;
  t.orientation.z = -0.062157;
  t.orientation.w = 0.846923;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.593042;
  t.position.y = -0.012532;
  t.position.z = 1.375272;
  t.orientation.x = 0.194362;
  t.orientation.y = 0.689974;
  t.orientation.z = 0.200846;
  t.orientation.w = 0.667698;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.525071;
  t.position.y = -0.320278;
  t.position.z = 1.265661;
  t.orientation.x = 0.339364;
  t.orientation.y = 0.494325;
  t.orientation.z = 0.217843;
  t.orientation.w = 0.770078;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.516629;
  t.position.y = -0.579264;
  t.position.z = 1.354172;
  t.orientation.x = 0.351622;
  t.orientation.y = 0.578856;
  t.orientation.z = 0.303746;
  t.orientation.w = 0.670095;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.297076;
  t.position.y = -0.225438;
  t.position.z = 1.373387;
  t.orientation.x = -0.282318;
  t.orientation.y = 0.375345;
  t.orientation.z = -0.121175;
  t.orientation.w = 0.874488;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.387618;
  t.position.y = -0.021124;
  t.position.z = 1.274235;
  t.orientation.x = 0.133552;
  t.orientation.y = 0.564296;
  t.orientation.z = 0.093114;
  t.orientation.w = 0.809360;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.687143;
  t.position.y = -0.076357;
  t.position.z = 1.333093;
  t.orientation.x = 0.300496;
  t.orientation.y = 0.601181;
  t.orientation.z = 0.260656;
  t.orientation.w = 0.693068;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.569265;
  t.position.y = -0.469885;
  t.position.z = 1.291283;
  t.orientation.x = -0.205665;
  t.orientation.y = 0.591325;
  t.orientation.z = -0.159324;
  t.orientation.w = 0.763317;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.437008;
  t.position.y = -0.149796;
  t.position.z = 1.332658;
  t.orientation.x = 0.054022;
  t.orientation.y = 0.649356;
  t.orientation.z = 0.046332;
  t.orientation.w = 0.757147;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.497110;
  t.position.y = -0.266490;
  t.position.z = 1.376124;
  t.orientation.x = -0.243011;
  t.orientation.y = 0.526380;
  t.orientation.z = -0.160116;
  t.orientation.w = 0.798895;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.265697;
  t.position.y = -0.609852;
  t.position.z = 1.314200;
  t.orientation.x = 0.301538;
  t.orientation.y = 0.354820;
  t.orientation.z = 0.122065;
  t.orientation.w = 0.876514;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.331355;
  t.position.y = -0.490987;
  t.position.z = 1.276734;
  t.orientation.x = -0.083609;
  t.orientation.y = 0.303249;
  t.orientation.z = -0.026721;
  t.orientation.w = 0.948860;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.536472;
  t.position.y = 0.067847;
  t.position.z = 1.317492;
  t.orientation.x = 0.289550;
  t.orientation.y = 0.505426;
  t.orientation.z = 0.184890;
  t.orientation.w = 0.791531;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.624947;
  t.position.y = -0.305692;
  t.position.z = 1.341347;
  t.orientation.x = -0.238827;
  t.orientation.y = 0.625533;
  t.orientation.z = -0.209665;
  t.orientation.w = 0.712538;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.583786;
  t.position.y = -0.122077;
  t.position.z = 1.388834;
  t.orientation.x = -0.275965;
  t.orientation.y = 0.647372;
  t.orientation.z = -0.272240;
  t.orientation.w = 0.656230;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.555342;
  t.position.y = -0.044623;
  t.position.z = 1.271320;
  t.orientation.x = -0.231152;
  t.orientation.y = 0.381004;
  t.orientation.z = -0.098986;
  t.orientation.w = 0.889723;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.430814;
  t.position.y = -0.043511;
  t.position.z = 1.396573;
  t.orientation.x = 0.136261;
  t.orientation.y = 0.474950;
  t.orientation.z = 0.074715;
  t.orientation.w = 0.866183;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.668862;
  t.position.y = 0.055605;
  t.position.z = 1.280562;
  t.orientation.x = -0.396132;
  t.orientation.y = 0.464820;
  t.orientation.z = -0.244476;
  t.orientation.w = 0.753162;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.609593;
  t.position.y = -0.191296;
  t.position.z = 1.276488;
  t.orientation.x = 0.257001;
  t.orientation.y = 0.426587;
  t.orientation.z = 0.127823;
  t.orientation.w = 0.857692;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.378772;
  t.position.y = 0.194389;
  t.position.z = 1.311157;
  t.orientation.x = 0.238511;
  t.orientation.y = 0.605633;
  t.orientation.z = 0.197028;
  t.orientation.w = 0.733145;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.365358;
  t.position.y = -0.371427;
  t.position.z = 1.290947;
  t.orientation.x = 0.401170;
  t.orientation.y = 0.502667;
  t.orientation.z = 0.283479;
  t.orientation.w = 0.711357;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.191345;
  t.position.y = -0.579851;
  t.position.z = 1.384156;
  t.orientation.x = 0.420752;
  t.orientation.y = 0.408589;
  t.orientation.z = 0.220592;
  t.orientation.w = 0.779334;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.224437;
  t.position.y = -0.351224;
  t.position.z = 1.336913;
  t.orientation.x = -0.111958;
  t.orientation.y = 0.390506;
  t.orientation.z = -0.047912;
  t.orientation.w = 0.912510;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.456092;
  t.position.y = -0.429910;
  t.position.z = 1.280075;
  t.orientation.x = -0.098749;
  t.orientation.y = 0.397676;
  t.orientation.z = -0.043098;
  t.orientation.w = 0.911178;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.681463;
  t.position.y = -0.460491;
  t.position.z = 1.254334;
  t.orientation.x = 0.277326;
  t.orientation.y = 0.672177;
  t.orientation.z = 0.302493;
  t.orientation.w = 0.616253;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.197744;
  t.position.y = -0.435590;
  t.position.z = 1.280015;
  t.orientation.x = 0.415653;
  t.orientation.y = 0.398088;
  t.orientation.z = 0.209309;
  t.orientation.w = 0.790537;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.378822;
  t.position.y = -0.608577;
  t.position.z = 1.380770;
  t.orientation.x = 0.173829;
  t.orientation.y = 0.341289;
  t.orientation.z = 0.064380;
  t.orientation.w = 0.921499;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.518320;
  t.position.y = -0.367548;
  t.position.z = 1.364679;
  t.orientation.x = 0.166492;
  t.orientation.y = 0.618533;
  t.orientation.z = 0.136266;
  t.orientation.w = 0.755731;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.430617;
  t.position.y = -0.410914;
  t.position.z = 1.392730;
  t.orientation.x = 0.138643;
  t.orientation.y = 0.358446;
  t.orientation.z = 0.053922;
  t.orientation.w = 0.921622;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.340062;
  t.position.y = -0.129281;
  t.position.z = 1.262261;
  t.orientation.x = -0.033157;
  t.orientation.y = 0.381727;
  t.orientation.z = -0.013704;
  t.orientation.w = 0.923578;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.361650;
  t.position.y = 0.315542;
  t.position.z = 1.264320;
  t.orientation.x = 0.003954;
  t.orientation.y = 0.328802;
  t.orientation.z = 0.001376;
  t.orientation.w = 0.944389;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.166058;
  t.position.y = -0.587556;
  t.position.z = 1.285312;
  t.orientation.x = 0.417966;
  t.orientation.y = 0.428698;
  t.orientation.z = 0.233907;
  t.orientation.w = 0.766035;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.400347;
  t.position.y = -0.694681;
  t.position.z = 1.300241;
  t.orientation.x = 0.384092;
  t.orientation.y = 0.484074;
  t.orientation.z = 0.249357;
  t.orientation.w = 0.745632;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.301600;
  t.position.y = 0.081603;
  t.position.z = 1.312188;
  t.orientation.x = 0.287972;
  t.orientation.y = 0.343573;
  t.orientation.z = 0.111557;
  t.orientation.w = 0.886896;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.422330;
  t.position.y = -0.223438;
  t.position.z = 1.254274;
  t.orientation.x = 0.279039;
  t.orientation.y = 0.416464;
  t.orientation.z = 0.135994;
  t.orientation.w = 0.854518;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.395547;
  t.position.y = 0.071057;
  t.position.z = 1.398339;
  t.orientation.x = 0.175201;
  t.orientation.y = 0.411344;
  t.orientation.z = 0.080901;
  t.orientation.w = 0.890818;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.272984;
  t.position.y = -0.465915;
  t.position.z = 1.393110;
  t.orientation.x = 0.295483;
  t.orientation.y = 0.555551;
  t.orientation.z = 0.220240;
  t.orientation.w = 0.745351;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.633405;
  t.position.y = 0.170807;
  t.position.z = 1.295378;
  t.orientation.x = -0.392656;
  t.orientation.y = 0.493591;
  t.orientation.z = -0.265840;
  t.orientation.w = 0.729053;
  sampling_poses.poses.push_back(t);

  sposes.push_back(sampling_poses);

  sampling_poses.poses.clear();

  t.position.x = 0.479544;
  t.position.y = -0.506505;
  t.position.z = 1.338098;
  t.orientation.x = 0.369059;
  t.orientation.y = 0.467425;
  t.orientation.z = 0.223579;
  t.orientation.w = 0.771571;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.551856;
  t.position.y = -0.122569;
  t.position.z = 1.317664;
  t.orientation.x = 0.162497;
  t.orientation.y = 0.618584;
  t.orientation.z = 0.132753;
  t.orientation.w = 0.757182;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.587574;
  t.position.y = 0.008379;
  t.position.z = 1.321428;
  t.orientation.x = -0.325653;
  t.orientation.y = 0.603470;
  t.orientation.z = -0.295432;
  t.orientation.w = 0.665202;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.450990;
  t.position.y = -0.194560;
  t.position.z = 1.311486;
  t.orientation.x = 0.207754;
  t.orientation.y = 0.348556;
  t.orientation.z = 0.079531;
  t.orientation.w = 0.910506;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.485703;
  t.position.y = 0.114441;
  t.position.z = 1.313777;
  t.orientation.x = 0.234290;
  t.orientation.y = 0.538730;
  t.orientation.z = 0.159075;
  t.orientation.w = 0.793456;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.186315;
  t.position.y = -0.473552;
  t.position.z = 1.307729;
  t.orientation.x = 0.151581;
  t.orientation.y = 0.349902;
  t.orientation.z = 0.057485;
  t.orientation.w = 0.922652;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.375945;
  t.position.y = -0.565794;
  t.position.z = 1.357858;
  t.orientation.x = -0.136164;
  t.orientation.y = 0.363204;
  t.orientation.z = -0.053748;
  t.orientation.w = 0.920138;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.349328;
  t.position.y = -0.416390;
  t.position.z = 1.340353;
  t.orientation.x = 0.236978;
  t.orientation.y = 0.515694;
  t.orientation.z = 0.150989;
  t.orientation.w = 0.809385;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.446334;
  t.position.y = -0.060920;
  t.position.z = 1.327192;
  t.orientation.x = 0.354462;
  t.orientation.y = 0.394665;
  t.orientation.z = 0.168383;
  t.orientation.w = 0.830809;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.643186;
  t.position.y = -0.198312;
  t.position.z = 1.258661;
  t.orientation.x = -0.259417;
  t.orientation.y = 0.511810;
  t.orientation.z = -0.165532;
  t.orientation.w = 0.802093;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.674125;
  t.position.y = 0.103442;
  t.position.z = 1.260449;
  t.orientation.x = 0.135255;
  t.orientation.y = 0.533026;
  t.orientation.z = 0.086788;
  t.orientation.w = 0.830697;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.402729;
  t.position.y = -0.696539;
  t.position.z = 1.359962;
  t.orientation.x = 0.307053;
  t.orientation.y = 0.523880;
  t.orientation.z = 0.209919;
  t.orientation.w = 0.766291;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.177393;
  t.position.y = -0.637636;
  t.position.z = 1.270808;
  t.orientation.x = 0.466262;
  t.orientation.y = 0.363346;
  t.orientation.z = 0.218172;
  t.orientation.w = 0.776518;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.389942;
  t.position.y = -0.640088;
  t.position.z = 1.265726;
  t.orientation.x = 0.371619;
  t.orientation.y = 0.480425;
  t.orientation.z = 0.235297;
  t.orientation.w = 0.758766;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.572848;
  t.position.y = -0.594384;
  t.position.z = 1.273530;
  t.orientation.x = 0.373121;
  t.orientation.y = 0.603252;
  t.orientation.z = 0.378532;
  t.orientation.w = 0.594627;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.265574;
  t.position.y = 0.295867;
  t.position.z = 1.352069;
  t.orientation.x = 0.164047;
  t.orientation.y = 0.400309;
  t.orientation.z = 0.073079;
  t.orientation.w = 0.898611;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.633913;
  t.position.y = -0.083596;
  t.position.z = 1.375351;
  t.orientation.x = -0.220704;
  t.orientation.y = 0.492584;
  t.orientation.z = -0.130730;
  t.orientation.w = 0.831601;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.555256;
  t.position.y = -0.351386;
  t.position.z = 1.293790;
  t.orientation.x = -0.062929;
  t.orientation.y = 0.393497;
  t.orientation.z = -0.027010;
  t.orientation.w = 0.916772;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.698184;
  t.position.y = -0.290222;
  t.position.z = 1.311585;
  t.orientation.x = 0.212894;
  t.orientation.y = 0.482570;
  t.orientation.z = 0.122195;
  t.orientation.w = 0.840756;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.287956;
  t.position.y = -0.263848;
  t.position.z = 1.351756;
  t.orientation.x = 0.135854;
  t.orientation.y = 0.455993;
  t.orientation.z = 0.070660;
  t.orientation.w = 0.876710;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.359153;
  t.position.y = 0.246911;
  t.position.z = 1.306891;
  t.orientation.x = 0.209669;
  t.orientation.y = 0.580091;
  t.orientation.z = 0.157724;
  t.orientation.w = 0.771140;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.230240;
  t.position.y = -0.568737;
  t.position.z = 1.321496;
  t.orientation.x = 0.406018;
  t.orientation.y = 0.439372;
  t.orientation.z = 0.232646;
  t.orientation.w = 0.766797;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.579543;
  t.position.y = 0.148874;
  t.position.z = 1.261886;
  t.orientation.x = -0.416545;
  t.orientation.y = 0.478842;
  t.orientation.z = -0.276384;
  t.orientation.w = 0.721674;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.300554;
  t.position.y = -0.133545;
  t.position.z = 1.389453;
  t.orientation.x = 0.191917;
  t.orientation.y = 0.490629;
  t.orientation.z = 0.111750;
  t.orientation.w = 0.842593;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.313484;
  t.position.y = -0.342575;
  t.position.z = 1.282065;
  t.orientation.x = -0.281018;
  t.orientation.y = 0.462009;
  t.orientation.z = -0.157111;
  t.orientation.w = 0.826374;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.174415;
  t.position.y = 0.186287;
  t.position.z = 1.273304;
  t.orientation.x = 0.244825;
  t.orientation.y = 0.419560;
  t.orientation.z = 0.118612;
  t.orientation.w = 0.866003;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.647025;
  t.position.y = -0.369958;
  t.position.z = 1.373969;
  t.orientation.x = -0.128687;
  t.orientation.y = 0.453301;
  t.orientation.z = -0.066325;
  t.orientation.w = 0.879522;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.666262;
  t.position.y = -0.437768;
  t.position.z = 1.297081;
  t.orientation.x = 0.293251;
  t.orientation.y = 0.657403;
  t.orientation.z = 0.310542;
  t.orientation.w = 0.620798;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.492272;
  t.position.y = 0.014086;
  t.position.z = 1.390494;
  t.orientation.x = -0.308064;
  t.orientation.y = 0.547303;
  t.orientation.z = -0.226469;
  t.orientation.w = 0.744491;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.548393;
  t.position.y = -0.232250;
  t.position.z = 1.278479;
  t.orientation.x = 0.243076;
  t.orientation.y = 0.585087;
  t.orientation.z = 0.189604;
  t.orientation.w = 0.750092;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.443320;
  t.position.y = -0.289185;
  t.position.z = 1.259595;
  t.orientation.x = -0.331288;
  t.orientation.y = 0.404726;
  t.orientation.z = -0.160167;
  t.orientation.w = 0.837133;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.390378;
  t.position.y = 0.051833;
  t.position.z = 1.305789;
  t.orientation.x = -0.403576;
  t.orientation.y = 0.437842;
  t.orientation.z = -0.229514;
  t.orientation.w = 0.769898;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.306488;
  t.position.y = -0.131511;
  t.position.z = 1.275471;
  t.orientation.x = 0.139464;
  t.orientation.y = 0.395432;
  t.orientation.z = 0.060884;
  t.orientation.w = 0.905801;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.495341;
  t.position.y = -0.625591;
  t.position.z = 1.370504;
  t.orientation.x = 0.070400;
  t.orientation.y = 0.451110;
  t.orientation.z = 0.035725;
  t.orientation.w = 0.888970;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.361801;
  t.position.y = 0.095709;
  t.position.z = 1.392552;
  t.orientation.x = 0.182209;
  t.orientation.y = 0.490317;
  t.orientation.z = 0.105639;
  t.orientation.w = 0.845712;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.475259;
  t.position.y = -0.388161;
  t.position.z = 1.364067;
  t.orientation.x = 0.322332;
  t.orientation.y = 0.570830;
  t.orientation.z = 0.259448;
  t.orientation.w = 0.709184;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.506628;
  t.position.y = -0.295151;
  t.position.z = 1.388559;
  t.orientation.x = 0.067896;
  t.orientation.y = 0.416392;
  t.orientation.z = 0.031201;
  t.orientation.w = 0.906109;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.477201;
  t.position.y = 0.282516;
  t.position.z = 1.314177;
  t.orientation.x = 0.028530;
  t.orientation.y = 0.556276;
  t.orientation.z = 0.019114;
  t.orientation.w = 0.830288;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.295112;
  t.position.y = -0.495001;
  t.position.z = 1.287148;
  t.orientation.x = 0.386755;
  t.orientation.y = 0.370146;
  t.orientation.z = 0.173166;
  t.orientation.w = 0.826696;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.259460;
  t.position.y = -0.455361;
  t.position.z = 1.378189;
  t.orientation.x = 0.072415;
  t.orientation.y = 0.342166;
  t.orientation.z = 0.026459;
  t.orientation.w = 0.936471;
  sampling_poses.poses.push_back(t);

  sposes.push_back(sampling_poses);

  sampling_poses.poses.clear();

  t.position.x = 0.231927;
  t.position.y = -0.222742;
  t.position.z = 1.347563;
  t.orientation.x = -0.173049;
  t.orientation.y = 0.505685;
  t.orientation.z = -0.104335;
  t.orientation.w = 0.838720;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.621615;
  t.position.y = -0.125615;
  t.position.z = 1.297141;
  t.orientation.x = -0.239710;
  t.orientation.y = 0.499720;
  t.orientation.z = -0.146187;
  t.orientation.w = 0.819419;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.472558;
  t.position.y = -0.473989;
  t.position.z = 1.333640;
  t.orientation.x = 0.370165;
  t.orientation.y = 0.477690;
  t.orientation.z = 0.231987;
  t.orientation.w = 0.762215;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.406252;
  t.position.y = -0.392988;
  t.position.z = 1.324987;
  t.orientation.x = -0.118194;
  t.orientation.y = 0.421889;
  t.orientation.z = -0.055579;
  t.orientation.w = 0.897191;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.509830;
  t.position.y = -0.366376;
  t.position.z = 1.257128;
  t.orientation.x = 0.155277;
  t.orientation.y = 0.657082;
  t.orientation.z = 0.140912;
  t.orientation.w = 0.724069;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.193889;
  t.position.y = -0.452734;
  t.position.z = 1.261944;
  t.orientation.x = 0.458946;
  t.orientation.y = 0.364209;
  t.orientation.z = 0.213842;
  t.orientation.w = 0.781660;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.482803;
  t.position.y = -0.162088;
  t.position.z = 1.271958;
  t.orientation.x = 0.325819;
  t.orientation.y = 0.596695;
  t.orientation.z = 0.288325;
  t.orientation.w = 0.674290;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.260567;
  t.position.y = 0.081377;
  t.position.z = 1.259948;
  t.orientation.x = 0.319492;
  t.orientation.y = 0.469688;
  t.orientation.z = 0.187247;
  t.orientation.w = 0.801409;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.633785;
  t.position.y = -0.369329;
  t.position.z = 1.256954;
  t.orientation.x = 0.029418;
  t.orientation.y = 0.752866;
  t.orientation.z = 0.033728;
  t.orientation.w = 0.656651;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.269151;
  t.position.y = -0.447520;
  t.position.z = 1.370603;
  t.orientation.x = 0.129009;
  t.orientation.y = 0.513120;
  t.orientation.z = 0.078345;
  t.orientation.w = 0.844942;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.384486;
  t.position.y = -0.696975;
  t.position.z = 1.336506;
  t.orientation.x = 0.055386;
  t.orientation.y = 0.369079;
  t.orientation.z = 0.022040;
  t.orientation.w = 0.927484;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.504421;
  t.position.y = 0.302547;
  t.position.z = 1.312208;
  t.orientation.x = -0.054665;
  t.orientation.y = 0.423811;
  t.orientation.z = -0.025636;
  t.orientation.w = 0.903736;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.534028;
  t.position.y = -0.661632;
  t.position.z = 1.262418;
  t.orientation.x = 0.411641;
  t.orientation.y = 0.493896;
  t.orientation.z = 0.286173;
  t.orientation.w = 0.710439;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.168853;
  t.position.y = -0.680510;
  t.position.z = 1.268231;
  t.orientation.x = 0.329709;
  t.orientation.y = 0.390277;
  t.orientation.z = 0.152088;
  t.orientation.w = 0.846076;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.596884;
  t.position.y = -0.537559;
  t.position.z = 1.338110;
  t.orientation.x = 0.219979;
  t.orientation.y = 0.552511;
  t.orientation.z = 0.154033;
  t.orientation.w = 0.789059;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.581180;
  t.position.y = 0.076986;
  t.position.z = 1.321155;
  t.orientation.x = 0.289858;
  t.orientation.y = 0.422706;
  t.orientation.z = 0.144764;
  t.orientation.w = 0.846372;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.336577;
  t.position.y = -0.203455;
  t.position.z = 1.364805;
  t.orientation.x = -0.290351;
  t.orientation.y = 0.365149;
  t.orientation.z = -0.121002;
  t.orientation.w = 0.876197;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.324741;
  t.position.y = -0.036403;
  t.position.z = 1.337378;
  t.orientation.x = 0.274181;
  t.orientation.y = 0.385465;
  t.orientation.z = 0.121106;
  t.orientation.w = 0.872682;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.291680;
  t.position.y = -0.609681;
  t.position.z = 1.336483;
  t.orientation.x = 0.418658;
  t.orientation.y = 0.445715;
  t.orientation.z = 0.248391;
  t.orientation.w = 0.751243;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.340819;
  t.position.y = 0.316299;
  t.position.z = 1.300545;
  t.orientation.x = 0.158524;
  t.orientation.y = 0.369875;
  t.orientation.z = 0.064207;
  t.orientation.w = 0.913203;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.299117;
  t.position.y = -0.323938;
  t.position.z = 1.386862;
  t.orientation.x = 0.062921;
  t.orientation.y = 0.427822;
  t.orientation.z = 0.029871;
  t.orientation.w = 0.901176;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.361854;
  t.position.y = 0.094884;
  t.position.z = 1.313334;
  t.orientation.x = 0.198725;
  t.orientation.y = 0.484952;
  t.orientation.z = 0.114189;
  t.orientation.w = 0.843973;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.393855;
  t.position.y = -0.570428;
  t.position.z = 1.345265;
  t.orientation.x = 0.356225;
  t.orientation.y = 0.445653;
  t.orientation.z = 0.199253;
  t.orientation.w = 0.796740;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.441377;
  t.position.y = -0.023830;
  t.position.z = 1.265553;
  t.orientation.x = -0.249807;
  t.orientation.y = 0.443849;
  t.orientation.z = -0.130343;
  t.orientation.w = 0.850650;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.273298;
  t.position.y = 0.188483;
  t.position.z = 1.335090;
  t.orientation.x = 0.266295;
  t.orientation.y = 0.396700;
  t.orientation.z = 0.121418;
  t.orientation.w = 0.870042;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.597442;
  t.position.y = -0.040815;
  t.position.z = 1.353145;
  t.orientation.x = -0.303506;
  t.orientation.y = 0.576325;
  t.orientation.z = -0.243390;
  t.orientation.w = 0.718676;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.429178;
  t.position.y = -0.293706;
  t.position.z = 1.288414;
  t.orientation.x = 0.016539;
  t.orientation.y = 0.484160;
  t.orientation.z = 0.009154;
  t.orientation.w = 0.874776;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.662747;
  t.position.y = -0.328184;
  t.position.z = 1.367496;
  t.orientation.x = 0.111997;
  t.orientation.y = 0.438948;
  t.orientation.z = 0.055250;
  t.orientation.w = 0.889792;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.530675;
  t.position.y = -0.245386;
  t.position.z = 1.336041;
  t.orientation.x = -0.151536;
  t.orientation.y = 0.521260;
  t.orientation.z = -0.094657;
  t.orientation.w = 0.834485;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.212506;
  t.position.y = 0.279146;
  t.position.z = 1.278509;
  t.orientation.x = 0.137196;
  t.orientation.y = 0.516509;
  t.orientation.z = 0.084260;
  t.orientation.w = 0.841009;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.174026;
  t.position.y = -0.412992;
  t.position.z = 1.353192;
  t.orientation.x = 0.393140;
  t.orientation.y = 0.504377;
  t.orientation.z = 0.276407;
  t.orientation.w = 0.717387;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.193307;
  t.position.y = -0.549363;
  t.position.z = 1.294028;
  t.orientation.x = 0.453010;
  t.orientation.y = 0.420134;
  t.orientation.z = 0.255998;
  t.orientation.w = 0.743461;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.438452;
  t.position.y = 0.149006;
  t.position.z = 1.263530;
  t.orientation.x = 0.010036;
  t.orientation.y = 0.644017;
  t.orientation.z = 0.008450;
  t.orientation.w = 0.764899;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.514112;
  t.position.y = -0.591429;
  t.position.z = 1.396548;
  t.orientation.x = 0.072712;
  t.orientation.y = 0.396988;
  t.orientation.z = 0.031568;
  t.orientation.w = 0.914394;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.515845;
  t.position.y = 0.006812;
  t.position.z = 1.395131;
  t.orientation.x = -0.295575;
  t.orientation.y = 0.437707;
  t.orientation.z = -0.154962;
  t.orientation.w = 0.834886;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.505322;
  t.position.y = -0.374147;
  t.position.z = 1.357327;
  t.orientation.x = 0.342010;
  t.orientation.y = 0.563093;
  t.orientation.z = 0.275031;
  t.orientation.w = 0.700224;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.467627;
  t.position.y = 0.124095;
  t.position.z = 1.388987;
  t.orientation.x = 0.038594;
  t.orientation.y = 0.389869;
  t.orientation.z = 0.016356;
  t.orientation.w = 0.919916;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.631378;
  t.position.y = -0.209424;
  t.position.z = 1.355510;
  t.orientation.x = 0.027232;
  t.orientation.y = 0.501322;
  t.orientation.z = 0.015789;
  t.orientation.w = 0.864688;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.430288;
  t.position.y = -0.608978;
  t.position.z = 1.258519;
  t.orientation.x = 0.374805;
  t.orientation.y = 0.545012;
  t.orientation.z = 0.296530;
  t.orientation.w = 0.688878;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.426362;
  t.position.y = -0.138248;
  t.position.z = 1.372923;
  t.orientation.x = 0.168516;
  t.orientation.y = 0.548526;
  t.orientation.z = 0.113976;
  t.orientation.w = 0.811006;
  sampling_poses.poses.push_back(t);

  sposes.push_back(sampling_poses);

  sampling_poses.poses.clear();

  t.position.x = 0.535241;
  t.position.y = -0.124562;
  t.position.z = 1.309922;
  t.orientation.x = -0.290825;
  t.orientation.y = 0.638986;
  t.orientation.z = -0.284699;
  t.orientation.w = 0.652736;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.171573;
  t.position.y = -0.619211;
  t.position.z = 1.381438;
  t.orientation.x = -0.008905;
  t.orientation.y = 0.486891;
  t.orientation.z = -0.004964;
  t.orientation.w = 0.873403;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.167828;
  t.position.y = -0.516901;
  t.position.z = 1.261191;
  t.orientation.x = 0.385103;
  t.orientation.y = 0.316179;
  t.orientation.z = 0.142369;
  t.orientation.w = 0.855253;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.528470;
  t.position.y = -0.316922;
  t.position.z = 1.257291;
  t.orientation.x = -0.044211;
  t.orientation.y = 0.670801;
  t.orientation.z = -0.040119;
  t.orientation.w = 0.739231;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.481696;
  t.position.y = 0.225609;
  t.position.z = 1.377931;
  t.orientation.x = -0.013005;
  t.orientation.y = 0.447649;
  t.orientation.z = -0.006511;
  t.orientation.w = 0.894091;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.688383;
  t.position.y = -0.061122;
  t.position.z = 1.286905;
  t.orientation.x = 0.170506;
  t.orientation.y = 0.682378;
  t.orientation.z = 0.168481;
  t.orientation.w = 0.690581;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.588786;
  t.position.y = -0.467365;
  t.position.z = 1.366331;
  t.orientation.x = -0.150693;
  t.orientation.y = 0.469527;
  t.orientation.z = -0.081691;
  t.orientation.w = 0.866119;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.355166;
  t.position.y = -0.191527;
  t.position.z = 1.355166;
  t.orientation.x = 0.348770;
  t.orientation.y = 0.533956;
  t.orientation.z = 0.256408;
  t.orientation.w = 0.726295;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.453017;
  t.position.y = -0.529691;
  t.position.z = 1.308371;
  t.orientation.x = 0.014126;
  t.orientation.y = 0.630396;
  t.orientation.z = 0.011474;
  t.orientation.w = 0.776060;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.657689;
  t.position.y = -0.407192;
  t.position.z = 1.295610;
  t.orientation.x = 0.253155;
  t.orientation.y = 0.442626;
  t.orientation.z = 0.131816;
  t.orientation.w = 0.850070;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.660770;
  t.position.y = -0.130118;
  t.position.z = 1.357015;
  t.orientation.x = -0.224717;
  t.orientation.y = 0.457852;
  t.orientation.z = -0.120812;
  t.orientation.w = 0.851633;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.290418;
  t.position.y = -0.076598;
  t.position.z = 1.318219;
  t.orientation.x = 0.289150;
  t.orientation.y = 0.335681;
  t.orientation.z = 0.109078;
  t.orientation.w = 0.889839;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.302063;
  t.position.y = -0.396684;
  t.position.z = 1.282685;
  t.orientation.x = 0.330263;
  t.orientation.y = 0.404880;
  t.orientation.z = 0.159650;
  t.orientation.w = 0.837562;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.496206;
  t.position.y = -0.616669;
  t.position.z = 1.337935;
  t.orientation.x = 0.265762;
  t.orientation.y = 0.429546;
  t.orientation.z = 0.133892;
  t.orientation.w = 0.852604;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.341724;
  t.position.y = 0.181688;
  t.position.z = 1.265711;
  t.orientation.x = -0.025724;
  t.orientation.y = 0.416967;
  t.orientation.z = -0.011806;
  t.orientation.w = 0.908481;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.602500;
  t.position.y = -0.208384;
  t.position.z = 1.276822;
  t.orientation.x = 0.001563;
  t.orientation.y = 0.595933;
  t.orientation.z = 0.001160;
  t.orientation.w = 0.803032;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.268613;
  t.position.y = -0.644771;
  t.position.z = 1.255743;
  t.orientation.x = 0.450713;
  t.orientation.y = 0.391150;
  t.orientation.z = 0.229267;
  t.orientation.w = 0.768958;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.415686;
  t.position.y = 0.139871;
  t.position.z = 1.348628;
  t.orientation.x = 0.273549;
  t.orientation.y = 0.451269;
  t.orientation.z = 0.147570;
  t.orientation.w = 0.836511;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.601660;
  t.position.y = 0.064695;
  t.position.z = 1.252176;
  t.orientation.x = -0.294224;
  t.orientation.y = 0.418891;
  t.orientation.z = -0.145576;
  t.orientation.w = 0.846623;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.424735;
  t.position.y = -0.646825;
  t.position.z = 1.260307;
  t.orientation.x = 0.369981;
  t.orientation.y = 0.395005;
  t.orientation.z = 0.177820;
  t.orientation.w = 0.821867;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.308060;
  t.position.y = -0.522557;
  t.position.z = 1.382369;
  t.orientation.x = -0.004402;
  t.orientation.y = 0.470949;
  t.orientation.z = -0.002350;
  t.orientation.w = 0.882147;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.515340;
  t.position.y = -0.028218;
  t.position.z = 1.358626;
  t.orientation.x = -0.245613;
  t.orientation.y = 0.561858;
  t.orientation.z = -0.179385;
  t.orientation.w = 0.769292;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.604319;
  t.position.y = 0.048224;
  t.position.z = 1.383862;
  t.orientation.x = -0.138712;
  t.orientation.y = 0.552666;
  t.orientation.z = -0.093902;
  t.orientation.w = 0.816395;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.419648;
  t.position.y = -0.232162;
  t.position.z = 1.256273;
  t.orientation.x = 0.046659;
  t.orientation.y = 0.502924;
  t.orientation.z = 0.027203;
  t.orientation.w = 0.862641;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.204243;
  t.position.y = -0.480709;
  t.position.z = 1.352488;
  t.orientation.x = -0.054353;
  t.orientation.y = 0.338397;
  t.orientation.z = -0.019583;
  t.orientation.w = 0.939228;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.309983;
  t.position.y = -0.665409;
  t.position.z = 1.373822;
  t.orientation.x = -0.025284;
  t.orientation.y = 0.390789;
  t.orientation.z = -0.010739;
  t.orientation.w = 0.920070;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.441028;
  t.position.y = -0.297181;
  t.position.z = 1.383895;
  t.orientation.x = 0.237358;
  t.orientation.y = 0.402371;
  t.orientation.z = 0.108846;
  t.orientation.w = 0.877446;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.256753;
  t.position.y = -0.288405;
  t.position.z = 1.309863;
  t.orientation.x = 0.381548;
  t.orientation.y = 0.389166;
  t.orientation.z = 0.181395;
  t.orientation.w = 0.818576;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.257744;
  t.position.y = 0.066943;
  t.position.z = 1.287616;
  t.orientation.x = 0.174913;
  t.orientation.y = 0.388455;
  t.orientation.z = 0.075364;
  t.orientation.w = 0.901570;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.356390;
  t.position.y = -0.300965;
  t.position.z = 1.295250;
  t.orientation.x = 0.267817;
  t.orientation.y = 0.539117;
  t.orientation.z = 0.185926;
  t.orientation.w = 0.776569;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.629571;
  t.position.y = -0.519858;
  t.position.z = 1.258153;
  t.orientation.x = 0.405692;
  t.orientation.y = 0.480304;
  t.orientation.z = 0.266759;
  t.orientation.w = 0.730453;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.505324;
  t.position.y = 0.153013;
  t.position.z = 1.283081;
  t.orientation.x = 0.162165;
  t.orientation.y = 0.589802;
  t.orientation.z = 0.122375;
  t.orientation.w = 0.781576;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.254929;
  t.position.y = 0.255814;
  t.position.z = 1.259930;
  t.orientation.x = 0.076380;
  t.orientation.y = 0.361124;
  t.orientation.z = 0.029694;
  t.orientation.w = 0.928910;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.390882;
  t.position.y = -0.134139;
  t.position.z = 1.281128;
  t.orientation.x = 0.234147;
  t.orientation.y = 0.348901;
  t.orientation.z = 0.090478;
  t.orientation.w = 0.902916;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.325582;
  t.position.y = 0.299092;
  t.position.z = 1.330757;
  t.orientation.x = 0.194371;
  t.orientation.y = 0.511816;
  t.orientation.z = 0.120126;
  t.orientation.w = 0.828151;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.434855;
  t.position.y = -0.009324;
  t.position.z = 1.262847;
  t.orientation.x = 0.038103;
  t.orientation.y = 0.536415;
  t.orientation.z = 0.024253;
  t.orientation.w = 0.842745;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.677852;
  t.position.y = -0.278957;
  t.position.z = 1.301541;
  t.orientation.x = 0.344522;
  t.orientation.y = 0.569823;
  t.orientation.z = 0.284677;
  t.orientation.w = 0.689613;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.605110;
  t.position.y = -0.244419;
  t.position.z = 1.371891;
  t.orientation.x = 0.067739;
  t.orientation.y = 0.444768;
  t.orientation.z = 0.033759;
  t.orientation.w = 0.892442;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.407287;
  t.position.y = -0.452583;
  t.position.z = 1.263074;
  t.orientation.x = 0.339695;
  t.orientation.y = 0.424353;
  t.orientation.z = 0.175625;
  t.orientation.w = 0.820785;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.510436;
  t.position.y = -0.238755;
  t.position.z = 1.339743;
  t.orientation.x = 0.246332;
  t.orientation.y = 0.502030;
  t.orientation.z = 0.151734;
  t.orientation.w = 0.815023;
  sampling_poses.poses.push_back(t);

  sposes.push_back(sampling_poses);

  sampling_poses.poses.clear();

  t.position.x = 0.324492;
  t.position.y = -0.392412;
  t.position.z = 1.359516;
  t.orientation.x = 0.000094;
  t.orientation.y = 0.377382;
  t.orientation.z = 0.000038;
  t.orientation.w = 0.926058;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.386981;
  t.position.y = -0.131354;
  t.position.z = 1.396029;
  t.orientation.x = -0.199833;
  t.orientation.y = 0.448011;
  t.orientation.z = -0.103471;
  t.orientation.w = 0.865244;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.685473;
  t.position.y = 0.060564;
  t.position.z = 1.262931;
  t.orientation.x = 0.189483;
  t.orientation.y = 0.677174;
  t.orientation.z = 0.187056;
  t.orientation.w = 0.685960;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.226728;
  t.position.y = -0.672876;
  t.position.z = 1.357105;
  t.orientation.x = 0.071074;
  t.orientation.y = 0.395001;
  t.orientation.z = 0.030669;
  t.orientation.w = 0.915414;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.666830;
  t.position.y = -0.288057;
  t.position.z = 1.297436;
  t.orientation.x = 0.298803;
  t.orientation.y = 0.522551;
  t.orientation.z = 0.202114;
  t.orientation.w = 0.772533;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.455689;
  t.position.y = -0.389689;
  t.position.z = 1.356456;
  t.orientation.x = 0.088660;
  t.orientation.y = 0.488718;
  t.orientation.z = 0.050007;
  t.orientation.w = 0.866483;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.436139;
  t.position.y = -0.455814;
  t.position.z = 1.251211;
  t.orientation.x = 0.312115;
  t.orientation.y = 0.544672;
  t.orientation.z = 0.228456;
  t.orientation.w = 0.744127;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.369976;
  t.position.y = 0.331316;
  t.position.z = 1.312868;
  t.orientation.x = 0.034100;
  t.orientation.y = 0.404699;
  t.orientation.z = 0.015104;
  t.orientation.w = 0.913689;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.599641;
  t.position.y = -0.606794;
  t.position.z = 1.272347;
  t.orientation.x = 0.246021;
  t.orientation.y = 0.432390;
  t.orientation.z = 0.123899;
  t.orientation.w = 0.858581;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.431539;
  t.position.y = -0.259079;
  t.position.z = 1.251082;
  t.orientation.x = 0.394856;
  t.orientation.y = 0.376435;
  t.orientation.z = 0.181673;
  t.orientation.w = 0.818157;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.165147;
  t.position.y = -0.476717;
  t.position.z = 1.298254;
  t.orientation.x = -0.214475;
  t.orientation.y = 0.280772;
  t.orientation.z = -0.064524;
  t.orientation.w = 0.933276;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.502749;
  t.position.y = -0.077410;
  t.position.z = 1.300519;
  t.orientation.x = -0.171363;
  t.orientation.y = 0.439739;
  t.orientation.z = -0.085881;
  t.orientation.w = 0.877433;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.375583;
  t.position.y = -0.623759;
  t.position.z = 1.351139;
  t.orientation.x = 0.345943;
  t.orientation.y = 0.401372;
  t.orientation.z = 0.166997;
  t.orientation.w = 0.831466;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.478222;
  t.position.y = 0.152369;
  t.position.z = 1.276622;
  t.orientation.x = -0.067017;
  t.orientation.y = 0.354033;
  t.orientation.z = -0.025444;
  t.orientation.w = 0.932482;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.286668;
  t.position.y = -0.497670;
  t.position.z = 1.327828;
  t.orientation.x = 0.084979;
  t.orientation.y = 0.384970;
  t.orientation.z = 0.035624;
  t.orientation.w = 0.918318;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.647223;
  t.position.y = -0.092301;
  t.position.z = 1.369128;
  t.orientation.x = 0.248682;
  t.orientation.y = 0.632719;
  t.orientation.z = 0.225475;
  t.orientation.w = 0.697843;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.577030;
  t.position.y = -0.338688;
  t.position.z = 1.383500;
  t.orientation.x = -0.212809;
  t.orientation.y = 0.415915;
  t.orientation.z = -0.100764;
  t.orientation.w = 0.878393;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.508424;
  t.position.y = -0.526638;
  t.position.z = 1.304014;
  t.orientation.x = 0.063255;
  t.orientation.y = 0.693200;
  t.orientation.z = 0.061297;
  t.orientation.w = 0.715343;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.508951;
  t.position.y = -0.646062;
  t.position.z = 1.289146;
  t.orientation.x = 0.354502;
  t.orientation.y = 0.523137;
  t.orientation.z = 0.253178;
  t.orientation.w = 0.732500;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.446530;
  t.position.y = 0.197621;
  t.position.z = 1.374417;
  t.orientation.x = 0.195692;
  t.orientation.y = 0.499261;
  t.orientation.z = 0.116877;
  t.orientation.w = 0.835932;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.323368;
  t.position.y = -0.243290;
  t.position.z = 1.394946;
  t.orientation.x = 0.175911;
  t.orientation.y = 0.523661;
  t.orientation.z = 0.111512;
  t.orientation.w = 0.826075;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.507295;
  t.position.y = 0.284752;
  t.position.z = 1.287130;
  t.orientation.x = -0.201299;
  t.orientation.y = 0.424481;
  t.orientation.z = -0.097389;
  t.orientation.w = 0.877388;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.673955;
  t.position.y = -0.447794;
  t.position.z = 1.263600;
  t.orientation.x = 0.371467;
  t.orientation.y = 0.461634;
  t.orientation.z = 0.221403;
  t.orientation.w = 0.774524;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.542628;
  t.position.y = 0.127857;
  t.position.z = 1.380631;
  t.orientation.x = 0.191475;
  t.orientation.y = 0.411037;
  t.orientation.z = 0.088744;
  t.orientation.w = 0.886854;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.211442;
  t.position.y = -0.214631;
  t.position.z = 1.376441;
  t.orientation.x = 0.300708;
  t.orientation.y = 0.469082;
  t.orientation.z = 0.173713;
  t.orientation.w = 0.812010;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.336516;
  t.position.y = -0.356757;
  t.position.z = 1.258353;
  t.orientation.x = 0.383538;
  t.orientation.y = 0.496733;
  t.orientation.z = 0.259550;
  t.orientation.w = 0.734022;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.297649;
  t.position.y = -0.617535;
  t.position.z = 1.263004;
  t.orientation.x = 0.309187;
  t.orientation.y = 0.462295;
  t.orientation.z = 0.175980;
  t.orientation.w = 0.812230;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.365218;
  t.position.y = -0.000165;
  t.position.z = 1.344291;
  t.orientation.x = 0.200346;
  t.orientation.y = 0.451234;
  t.orientation.z = 0.104718;
  t.orientation.w = 0.863298;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.590933;
  t.position.y = -0.235484;
  t.position.z = 1.354161;
  t.orientation.x = -0.156222;
  t.orientation.y = 0.443094;
  t.orientation.z = -0.078728;
  t.orientation.w = 0.879241;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.610114;
  t.position.y = -0.190173;
  t.position.z = 1.264191;
  t.orientation.x = -0.355184;
  t.orientation.y = 0.415499;
  t.orientation.z = -0.180480;
  t.orientation.w = 0.817699;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.540271;
  t.position.y = 0.072206;
  t.position.z = 1.259922;
  t.orientation.x = 0.290647;
  t.orientation.y = 0.374476;
  t.orientation.z = 0.124873;
  t.orientation.w = 0.871607;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.373697;
  t.position.y = -0.192603;
  t.position.z = 1.317403;
  t.orientation.x = 0.119901;
  t.orientation.y = 0.447161;
  t.orientation.z = 0.060630;
  t.orientation.w = 0.884305;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.189348;
  t.position.y = -0.575701;
  t.position.z = 1.389646;
  t.orientation.x = 0.107766;
  t.orientation.y = 0.306731;
  t.orientation.z = 0.034978;
  t.orientation.w = 0.945029;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.453948;
  t.position.y = -0.276606;
  t.position.z = 1.367183;
  t.orientation.x = 0.234200;
  t.orientation.y = 0.436032;
  t.orientation.z = 0.118634;
  t.orientation.w = 0.860786;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.330261;
  t.position.y = 0.105674;
  t.position.z = 1.306288;
  t.orientation.x = 0.121459;
  t.orientation.y = 0.423107;
  t.orientation.z = 0.057351;
  t.orientation.w = 0.896069;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.547560;
  t.position.y = -0.428584;
  t.position.z = 1.328545;
  t.orientation.x = -0.200098;
  t.orientation.y = 0.556429;
  t.orientation.z = -0.140198;
  t.orientation.w = 0.794161;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.592517;
  t.position.y = -0.353214;
  t.position.z = 1.275170;
  t.orientation.x = 0.337000;
  t.orientation.y = 0.587832;
  t.orientation.z = 0.293828;
  t.orientation.w = 0.674203;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.601950;
  t.position.y = -0.053290;
  t.position.z = 1.255825;
  t.orientation.x = 0.276981;
  t.orientation.y = 0.660150;
  t.orientation.z = 0.287350;
  t.orientation.w = 0.636329;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.409354;
  t.position.y = -0.691903;
  t.position.z = 1.284573;
  t.orientation.x = 0.331140;
  t.orientation.y = 0.346304;
  t.orientation.z = 0.132155;
  t.orientation.w = 0.867730;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.184037;
  t.position.y = -0.574203;
  t.position.z = 1.260495;
  t.orientation.x = 0.394557;
  t.orientation.y = 0.391646;
  t.orientation.z = 0.191014;
  t.orientation.w = 0.808982;
  sampling_poses.poses.push_back(t);

  sposes.push_back(sampling_poses);

  sampling_poses.poses.clear();

  t.position.x = 0.523234;
  t.position.y = -0.361312;
  t.position.z = 1.336847;
  t.orientation.x = 0.050365;
  t.orientation.y = 0.646932;
  t.orientation.z = 0.042891;
  t.orientation.w = 0.759673;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.354695;
  t.position.y = -0.598192;
  t.position.z = 1.337816;
  t.orientation.x = 0.189435;
  t.orientation.y = 0.537662;
  t.orientation.z = 0.125438;
  t.orientation.w = 0.811972;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.224979;
  t.position.y = -0.090363;
  t.position.z = 1.363534;
  t.orientation.x = -0.302062;
  t.orientation.y = 0.517783;
  t.orientation.z = -0.201935;
  t.orientation.w = 0.774520;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.260931;
  t.position.y = -0.263446;
  t.position.z = 1.358612;
  t.orientation.x = 0.014753;
  t.orientation.y = 0.298727;
  t.orientation.z = 0.004619;
  t.orientation.w = 0.954213;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.326014;
  t.position.y = -0.390802;
  t.position.z = 1.291526;
  t.orientation.x = -0.110558;
  t.orientation.y = 0.459381;
  t.orientation.z = -0.057751;
  t.orientation.w = 0.879438;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.518393;
  t.position.y = 0.017889;
  t.position.z = 1.356287;
  t.orientation.x = 0.261774;
  t.orientation.y = 0.389458;
  t.orientation.z = 0.116468;
  t.orientation.w = 0.875347;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.584930;
  t.position.y = 0.208319;
  t.position.z = 1.284775;
  t.orientation.x = 0.201631;
  t.orientation.y = 0.403659;
  t.orientation.z = 0.091688;
  t.orientation.w = 0.887692;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.395201;
  t.position.y = -0.282792;
  t.position.z = 1.308488;
  t.orientation.x = 0.037107;
  t.orientation.y = 0.317220;
  t.orientation.z = 0.012423;
  t.orientation.w = 0.947544;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.430291;
  t.position.y = -0.540313;
  t.position.z = 1.266525;
  t.orientation.x = 0.297420;
  t.orientation.y = 0.435127;
  t.orientation.z = 0.154878;
  t.orientation.w = 0.835595;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.447126;
  t.position.y = -0.205135;
  t.position.z = 1.365213;
  t.orientation.x = -0.203257;
  t.orientation.y = 0.408970;
  t.orientation.z = -0.093965;
  t.orientation.w = 0.884647;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.359013;
  t.position.y = -0.140288;
  t.position.z = 1.357289;
  t.orientation.x = -0.226773;
  t.orientation.y = 0.520695;
  t.orientation.z = -0.145766;
  t.orientation.w = 0.810063;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.520459;
  t.position.y = -0.123416;
  t.position.z = 1.377616;
  t.orientation.x = 0.143219;
  t.orientation.y = 0.556483;
  t.orientation.z = 0.098088;
  t.orientation.w = 0.812523;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.475575;
  t.position.y = -0.650064;
  t.position.z = 1.314518;
  t.orientation.x = 0.427193;
  t.orientation.y = 0.426003;
  t.orientation.z = 0.239205;
  t.orientation.w = 0.760794;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.559187;
  t.position.y = -0.070771;
  t.position.z = 1.298696;
  t.orientation.x = -0.204403;
  t.orientation.y = 0.511452;
  t.orientation.z = -0.126723;
  t.orientation.w = 0.824971;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.507288;
  t.position.y = -0.466547;
  t.position.z = 1.358076;
  t.orientation.x = 0.388730;
  t.orientation.y = 0.435301;
  t.orientation.z = 0.216186;
  t.orientation.w = 0.782729;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.406981;
  t.position.y = -0.044479;
  t.position.z = 1.322546;
  t.orientation.x = 0.041757;
  t.orientation.y = 0.632549;
  t.orientation.z = 0.034186;
  t.orientation.w = 0.772638;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.370710;
  t.position.y = -0.694711;
  t.position.z = 1.381718;
  t.orientation.x = 0.383844;
  t.orientation.y = 0.571476;
  t.orientation.z = 0.343333;
  t.orientation.w = 0.638906;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.676408;
  t.position.y = -0.444573;
  t.position.z = 1.253935;
  t.orientation.x = -0.076993;
  t.orientation.y = 0.599610;
  t.orientation.z = -0.058110;
  t.orientation.w = 0.794458;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.648779;
  t.position.y = -0.293013;
  t.position.z = 1.328713;
  t.orientation.x = 0.294792;
  t.orientation.y = 0.443519;
  t.orientation.z = 0.157209;
  t.orientation.w = 0.831669;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.542269;
  t.position.y = 0.109416;
  t.position.z = 1.264652;
  t.orientation.x = -0.373097;
  t.orientation.y = 0.473605;
  t.orientation.z = -0.231434;
  t.orientation.w = 0.763502;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.575679;
  t.position.y = -0.599475;
  t.position.z = 1.272364;
  t.orientation.x = 0.320586;
  t.orientation.y = 0.431477;
  t.orientation.z = 0.167371;
  t.orientation.w = 0.826462;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.305795;
  t.position.y = 0.181084;
  t.position.z = 1.302091;
  t.orientation.x = 0.225276;
  t.orientation.y = 0.315924;
  t.orientation.z = 0.077495;
  t.orientation.w = 0.918388;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.419782;
  t.position.y = -0.518328;
  t.position.z = 1.376816;
  t.orientation.x = 0.292932;
  t.orientation.y = 0.595542;
  t.orientation.z = 0.247093;
  t.orientation.w = 0.706021;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.333067;
  t.position.y = 0.079105;
  t.position.z = 1.333450;
  t.orientation.x = 0.268771;
  t.orientation.y = 0.533276;
  t.orientation.z = 0.183562;
  t.orientation.w = 0.780822;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.640643;
  t.position.y = 0.093871;
  t.position.z = 1.289172;
  t.orientation.x = 0.298648;
  t.orientation.y = 0.452111;
  t.orientation.z = 0.163789;
  t.orientation.w = 0.824365;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.428627;
  t.position.y = 0.053944;
  t.position.z = 1.301696;
  t.orientation.x = -0.055958;
  t.orientation.y = 0.363003;
  t.orientation.z = -0.021845;
  t.orientation.w = 0.929850;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.264784;
  t.position.y = -0.462751;
  t.position.z = 1.390301;
  t.orientation.x = -0.041714;
  t.orientation.y = 0.348688;
  t.orientation.z = -0.015537;
  t.orientation.w = 0.936181;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.437786;
  t.position.y = 0.246167;
  t.position.z = 1.272145;
  t.orientation.x = -0.107473;
  t.orientation.y = 0.464150;
  t.orientation.z = -0.056856;
  t.orientation.w = 0.877372;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.211173;
  t.position.y = -0.482112;
  t.position.z = 1.274299;
  t.orientation.x = 0.311908;
  t.orientation.y = 0.453389;
  t.orientation.z = 0.173131;
  t.orientation.w = 0.816810;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.392611;
  t.position.y = 0.162320;
  t.position.z = 1.383845;
  t.orientation.x = 0.153990;
  t.orientation.y = 0.346145;
  t.orientation.z = 0.057709;
  t.orientation.w = 0.923656;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.493486;
  t.position.y = -0.143208;
  t.position.z = 1.262431;
  t.orientation.x = 0.302120;
  t.orientation.y = 0.565841;
  t.orientation.z = 0.233983;
  t.orientation.w = 0.730616;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.354851;
  t.position.y = 0.306142;
  t.position.z = 1.324087;
  t.orientation.x = 0.086681;
  t.orientation.y = 0.362538;
  t.orientation.z = 0.033889;
  t.orientation.w = 0.927310;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.208867;
  t.position.y = -0.639526;
  t.position.z = 1.277204;
  t.orientation.x = 0.425388;
  t.orientation.y = 0.477165;
  t.orientation.z = 0.284039;
  t.orientation.w = 0.714619;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.636980;
  t.position.y = -0.219960;
  t.position.z = 1.255025;
  t.orientation.x = -0.236318;
  t.orientation.y = 0.690199;
  t.orientation.z = -0.257404;
  t.orientation.w = 0.633658;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.216894;
  t.position.y = -0.685360;
  t.position.z = 1.376405;
  t.orientation.x = 0.242218;
  t.orientation.y = 0.482640;
  t.orientation.z = 0.140885;
  t.orientation.w = 0.829783;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.328454;
  t.position.y = -0.003189;
  t.position.z = 1.264046;
  t.orientation.x = 0.121672;
  t.orientation.y = 0.471202;
  t.orientation.z = 0.065815;
  t.orientation.w = 0.871110;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.638659;
  t.position.y = -0.395840;
  t.position.z = 1.373901;
  t.orientation.x = -0.032342;
  t.orientation.y = 0.581390;
  t.orientation.z = -0.023138;
  t.orientation.w = 0.812653;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.202723;
  t.position.y = -0.373395;
  t.position.z = 1.320728;
  t.orientation.x = 0.138239;
  t.orientation.y = 0.284999;
  t.orientation.z = 0.041577;
  t.orientation.w = 0.947595;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.548573;
  t.position.y = -0.234252;
  t.position.z = 1.366233;
  t.orientation.x = -0.257962;
  t.orientation.y = 0.607381;
  t.orientation.z = -0.217893;
  t.orientation.w = 0.719073;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.653051;
  t.position.y = -0.131753;
  t.position.z = 1.321120;
  t.orientation.x = -0.346572;
  t.orientation.y = 0.456423;
  t.orientation.z = -0.198981;
  t.orientation.w = 0.794967;
  sampling_poses.poses.push_back(t);

  sposes.push_back(sampling_poses);

  sampling_poses.poses.clear();

  t.position.x = 0.378993;
  t.position.y = -0.679738;
  t.position.z = 1.297064;
  t.orientation.x = 0.277249;
  t.orientation.y = 0.435906;
  t.orientation.z = 0.143164;
  t.orientation.w = 0.844170;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.606053;
  t.position.y = 0.011712;
  t.position.z = 1.338951;
  t.orientation.x = 0.134169;
  t.orientation.y = 0.718489;
  t.orientation.z = 0.144527;
  t.orientation.w = 0.666996;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.599602;
  t.position.y = -0.312613;
  t.position.z = 1.305888;
  t.orientation.x = -0.124081;
  t.orientation.y = 0.412920;
  t.orientation.z = -0.056898;
  t.orientation.w = 0.900480;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.508605;
  t.position.y = -0.517872;
  t.position.z = 1.284978;
  t.orientation.x = 0.344123;
  t.orientation.y = 0.595346;
  t.orientation.z = 0.312646;
  t.orientation.w = 0.655283;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.422588;
  t.position.y = 0.280727;
  t.position.z = 1.350929;
  t.orientation.x = 0.099397;
  t.orientation.y = 0.599552;
  t.orientation.z = 0.075382;
  t.orientation.w = 0.790553;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.282860;
  t.position.y = -0.391397;
  t.position.z = 1.345359;
  t.orientation.x = -0.095569;
  t.orientation.y = 0.513630;
  t.orientation.z = -0.057701;
  t.orientation.w = 0.850718;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.411274;
  t.position.y = -0.461142;
  t.position.z = 1.376482;
  t.orientation.x = 0.133296;
  t.orientation.y = 0.488403;
  t.orientation.z = 0.075785;
  t.orientation.w = 0.859041;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.485299;
  t.position.y = -0.421048;
  t.position.z = 1.267244;
  t.orientation.x = 0.087502;
  t.orientation.y = 0.522778;
  t.orientation.z = 0.054055;
  t.orientation.w = 0.846241;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.263732;
  t.position.y = 0.225210;
  t.position.z = 1.269539;
  t.orientation.x = 0.249703;
  t.orientation.y = 0.289536;
  t.orientation.z = 0.078527;
  t.orientation.w = 0.920680;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.233831;
  t.position.y = -0.150439;
  t.position.z = 1.336995;
  t.orientation.x = 0.249526;
  t.orientation.y = 0.323722;
  t.orientation.z = 0.088931;
  t.orientation.w = 0.908313;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.540122;
  t.position.y = -0.181814;
  t.position.z = 1.263950;
  t.orientation.x = 0.364090;
  t.orientation.y = 0.548736;
  t.orientation.z = 0.287228;
  t.orientation.w = 0.695577;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.517678;
  t.position.y = -0.038329;
  t.position.z = 1.266883;
  t.orientation.x = 0.176796;
  t.orientation.y = 0.466878;
  t.orientation.z = 0.095851;
  t.orientation.w = 0.861151;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.202649;
  t.position.y = -0.623078;
  t.position.z = 1.395246;
  t.orientation.x = 0.400969;
  t.orientation.y = 0.458282;
  t.orientation.z = 0.243401;
  t.orientation.w = 0.754955;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.584777;
  t.position.y = -0.469883;
  t.position.z = 1.340935;
  t.orientation.x = 0.346888;
  t.orientation.y = 0.444691;
  t.orientation.z = 0.192069;
  t.orientation.w = 0.803136;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.311961;
  t.position.y = -0.272789;
  t.position.z = 1.318182;
  t.orientation.x = 0.298279;
  t.orientation.y = 0.538258;
  t.orientation.z = 0.211434;
  t.orientation.w = 0.759345;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.497115;
  t.position.y = 0.103994;
  t.position.z = 1.290642;
  t.orientation.x = -0.014836;
  t.orientation.y = 0.478971;
  t.orientation.z = -0.008096;
  t.orientation.w = 0.877668;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.660445;
  t.position.y = -0.448885;
  t.position.z = 1.272274;
  t.orientation.x = 0.288587;
  t.orientation.y = 0.671084;
  t.orientation.z = 0.321417;
  t.orientation.w = 0.602540;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.576480;
  t.position.y = 0.186378;
  t.position.z = 1.294227;
  t.orientation.x = -0.125229;
  t.orientation.y = 0.398957;
  t.orientation.z = -0.055102;
  t.orientation.w = 0.906705;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.294202;
  t.position.y = -0.517366;
  t.position.z = 1.311958;
  t.orientation.x = 0.325945;
  t.orientation.y = 0.410660;
  t.orientation.z = 0.160041;
  t.orientation.w = 0.836364;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.335963;
  t.position.y = -0.068346;
  t.position.z = 1.341008;
  t.orientation.x = -0.324153;
  t.orientation.y = 0.493525;
  t.orientation.z = -0.204938;
  t.orientation.w = 0.780614;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.206386;
  t.position.y = -0.627350;
  t.position.z = 1.258452;
  t.orientation.x = 0.273423;
  t.orientation.y = 0.320793;
  t.orientation.z = 0.097286;
  t.orientation.w = 0.901591;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.523822;
  t.position.y = -0.254550;
  t.position.z = 1.340251;
  t.orientation.x = -0.230642;
  t.orientation.y = 0.577532;
  t.orientation.z = -0.174481;
  t.orientation.w = 0.763425;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.426343;
  t.position.y = -0.164109;
  t.position.z = 1.285663;
  t.orientation.x = -0.180179;
  t.orientation.y = 0.453969;
  t.orientation.z = -0.094289;
  t.orientation.w = 0.867501;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.652750;
  t.position.y = -0.062495;
  t.position.z = 1.274035;
  t.orientation.x = 0.265646;
  t.orientation.y = 0.663092;
  t.orientation.z = 0.273444;
  t.orientation.w = 0.644181;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.456769;
  t.position.y = -0.080287;
  t.position.z = 1.345272;
  t.orientation.x = 0.030608;
  t.orientation.y = 0.390418;
  t.orientation.z = 0.012988;
  t.orientation.w = 0.920037;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.407985;
  t.position.y = -0.045626;
  t.position.z = 1.263457;
  t.orientation.x = 0.187217;
  t.orientation.y = 0.514948;
  t.orientation.z = 0.116378;
  t.orientation.w = 0.828393;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.192781;
  t.position.y = -0.512550;
  t.position.z = 1.299579;
  t.orientation.x = 0.455737;
  t.orientation.y = 0.381296;
  t.orientation.z = 0.225036;
  t.orientation.w = 0.772189;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.689454;
  t.position.y = -0.199231;
  t.position.z = 1.308647;
  t.orientation.x = 0.128724;
  t.orientation.y = 0.751373;
  t.orientation.z = 0.153853;
  t.orientation.w = 0.628648;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.403549;
  t.position.y = 0.177123;
  t.position.z = 1.333572;
  t.orientation.x = 0.131109;
  t.orientation.y = 0.495412;
  t.orientation.z = 0.075938;
  t.orientation.w = 0.855343;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.394607;
  t.position.y = -0.178002;
  t.position.z = 1.380960;
  t.orientation.x = 0.270799;
  t.orientation.y = 0.611936;
  t.orientation.z = 0.235071;
  t.orientation.w = 0.704943;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.508472;
  t.position.y = 0.301895;
  t.position.z = 1.268702;
  t.orientation.x = 0.119332;
  t.orientation.y = 0.461778;
  t.orientation.z = 0.062856;
  t.orientation.w = 0.876681;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.506362;
  t.position.y = 0.195863;
  t.position.z = 1.365815;
  t.orientation.x = 0.137119;
  t.orientation.y = 0.476856;
  t.orientation.z = 0.075598;
  t.orientation.w = 0.864923;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.328042;
  t.position.y = -0.182458;
  t.position.z = 1.258696;
  t.orientation.x = 0.199805;
  t.orientation.y = 0.471262;
  t.orientation.z = 0.110527;
  t.orientation.w = 0.851924;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.505189;
  t.position.y = -0.346187;
  t.position.z = 1.395315;
  t.orientation.x = 0.382069;
  t.orientation.y = 0.480504;
  t.orientation.z = 0.244605;
  t.orientation.w = 0.750538;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.172237;
  t.position.y = -0.400198;
  t.position.z = 1.374854;
  t.orientation.x = 0.378893;
  t.orientation.y = 0.405187;
  t.orientation.z = 0.189497;
  t.orientation.w = 0.810157;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.410396;
  t.position.y = -0.303163;
  t.position.z = 1.339867;
  t.orientation.x = 0.026270;
  t.orientation.y = 0.571249;
  t.orientation.z = 0.018297;
  t.orientation.w = 0.820153;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.557877;
  t.position.y = -0.104097;
  t.position.z = 1.364756;
  t.orientation.x = 0.167822;
  t.orientation.y = 0.679910;
  t.orientation.z = 0.164254;
  t.orientation.w = 0.694678;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.383131;
  t.position.y = -0.458621;
  t.position.z = 1.275718;
  t.orientation.x = 0.249453;
  t.orientation.y = 0.317718;
  t.orientation.z = 0.087033;
  t.orientation.w = 0.910634;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.294611;
  t.position.y = 0.050566;
  t.position.z = 1.332523;
  t.orientation.x = 0.330168;
  t.orientation.y = 0.333219;
  t.orientation.z = 0.125859;
  t.orientation.w = 0.874136;
  sampling_poses.poses.push_back(t);
  t.position.x = 0.533175;
  t.position.y = -0.643576;
  t.position.z = 1.350516;
  t.orientation.x = 0.077588;
  t.orientation.y = 0.546569;
  t.orientation.z = 0.050954;
  t.orientation.w = 0.832254;
  sampling_poses.poses.push_back(t);

  sposes.push_back(sampling_poses);

  return sposes[i];
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
