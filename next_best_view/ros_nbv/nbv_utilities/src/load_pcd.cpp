/*
 * load_pcd.cpp
 *
 *  Created on: Dec 2, 2010
 *      Author: Christian Potthast
 */

#include <nbv_utilities/load_pcd.h>

LoadPcd::LoadPcd(ros::NodeHandle& n)
{
  tf_frame_  = ("/map");
  node_handle_ = n;
}

int LoadPcd::load(std::string& file_name)
{
  if (file_name == "" || pcl::io::loadPCDFile (file_name, cloud_, origin_, orientation_) == -1)
    return (-1);

  pcl::fromROSMsg (cloud_, pcl_cloud_);
  cloud_.header.stamp = ros::Time();
  cloud_.header.frame_id = tf_frame_;
  return (0);
}

int main(int argc, char** argv)
{
  if(argc == 3 || argc == 4){
    ros::init(argc, argv, "nbv_utilities_load_pcd");
    ros::NodeHandle n;
    ros::Rate loop_rate(5);

    LoadPcd loadPcd(n);
    std::string file_name = argv[1];
    std::string service_name = argv[2];

    float direction = -1.0;
    if(argc == 4){
      std::string dir = argv[3];
      direction = boost::lexical_cast<float>( dir.c_str() );
    }

    ROS_INFO("Loading: %s", file_name.c_str());
    ROS_INFO("Call service: %s", service_name.c_str());
    ROS_INFO("direction: %f", direction);

    if (loadPcd.load(file_name) == -1)
    {
      ROS_ERROR ("Could not load file. Exiting.");
      return (-1);
    }
    ROS_INFO ("Loaded a point cloud with %d points (total size is %d) and the following channels: %s.",
              loadPcd.cloud_.width * loadPcd.cloud_.height,
              (uint) loadPcd.cloud_.data.size (), pcl::getFieldsList (loadPcd.cloud_).c_str ());

    size_t found;
    found=service_name.find("SendCloud");
    if (found!=std::string::npos)
    {
/*
      size_t index_pos_begin;
      index_pos_begin = file_name.find("_");
      size_t index_pos_end;
      index_pos_end = file_name.find(".");
      std::string num = file_name.substr(index_pos_begin+1, index_pos_end-index_pos_begin-1);
      int index = boost::lexical_cast<int>( num.c_str() );
*/
      // Open file for reading
      std::ifstream in("scan_poses.txt");
      // Read and process data in file
      std::string lineData;
      std::vector<std::string> input;
      while (in)
      {
        in >> lineData;
        if ( !in )
        {
          if ( !in.eof() )
          {
            ROS_ERROR("ERROR reading file");
          }
        }
        else
        {
          input.push_back(lineData);
        }
      }
      in.close();

  //    int multi = (index * 3) - 3;
      geometry_msgs::PoseWithCovarianceStamped robot_pose;
      if(input.size() > 0)
      {
    //    robot_pose.pose.pose.position.x = boost::lexical_cast<float>(input[multi]);
    //    robot_pose.pose.pose.position.y = boost::lexical_cast<float>(input[multi+1]);
    //    robot_pose.pose.pose.position.z = boost::lexical_cast<float>(input[multi+2]);
        //ROS_INFO("%f %f %f", robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, robot_pose.pose.pose.position.z);
      }else{
        robot_pose.pose.pose.position.x = loadPcd.origin_.x();
        robot_pose.pose.pose.position.y = loadPcd.origin_.y();
        robot_pose.pose.pose.position.z = loadPcd.origin_.z();

        robot_pose.pose.pose.orientation.x = loadPcd.orientation_.x();
        robot_pose.pose.pose.orientation.y = loadPcd.orientation_.y();
        robot_pose.pose.pose.orientation.z = loadPcd.orientation_.z();
        robot_pose.pose.pose.orientation.w = loadPcd.orientation_.w();
      }

      //Call service to extract the objects
      nbv_utilities::SendCloud::Request request;
      nbv_utilities::SendCloud::Response response;

      request.cloud = loadPcd.cloud_;
      request.robot_pose = robot_pose;
      request.dir = direction;

      if (!ros::service::call(service_name, request, response)){
        ROS_ERROR("LoadPcd: error sending point cloud to extractObjects");
        return 0;
      }else{
        if(response.result == response.SUCCEEDED)
          ROS_INFO("LoadPcd : send cloud status: SUCCEEDED \n");
        else if(response.result == response.DONE)
          ROS_INFO("LoadPcd : send cloud status: DONE \n");
        else if(response.result == response.FAILED)
          ROS_INFO("LoadPcd : send cloud status: FAILED \n");
      }



    }

    found=service_name.find("FindTable");
    if (found!=std::string::npos)
    {
      //Call service to extract the objects
      nbv_utilities::FindTable::Request request;
      nbv_utilities::FindTable::Response response;

      std::ifstream in("init_poses.txt");
      // Read and process data in file
      std::string lineData;
      std::vector<std::string> input;
      while (in)
      {
        in >> lineData;
        if ( !in )
        {
          if ( !in.eof() )
          {
            ROS_ERROR("ERROR reading file");
          }
        }
        else
        {
          input.push_back(lineData);
        }
      }
      in.close();

      geometry_msgs::PoseWithCovarianceStamped robot_pose;
      if(input.size() > 0)
      {
        robot_pose.pose.pose.position.x = boost::lexical_cast<float>(input[0]);
        robot_pose.pose.pose.position.y = boost::lexical_cast<float>(input[1]);
        robot_pose.pose.pose.position.z = boost::lexical_cast<float>(input[2]);
      }else{
        robot_pose.pose.pose.position.x = loadPcd.origin_.x();
        robot_pose.pose.pose.position.y = loadPcd.origin_.y();
        robot_pose.pose.pose.position.z = loadPcd.origin_.z();

        robot_pose.pose.pose.orientation.x = loadPcd.orientation_.x();
        robot_pose.pose.pose.orientation.y = loadPcd.orientation_.y();
        robot_pose.pose.pose.orientation.z = loadPcd.orientation_.z();
        robot_pose.pose.pose.orientation.w = loadPcd.orientation_.w();
      }

      request.cloud = loadPcd.cloud_;
      request.robot_pose = robot_pose;

      if (!ros::service::call(service_name, request, response)){
        ROS_ERROR("LoadPcd: error sending point cloud to extractObjects");
        return 0;
      }else{
        if(response.result == response.SUCCEEDED)
          ROS_INFO("LoadPcd : send cloud status: SUCCEEDED \n");
        else if(response.result == response.FAILED)
          ROS_INFO("LoadPcd : send cloud status: FAILED \n");
      }

      ROS_INFO("origin: %f %f %f ; orientation: %f %f %f %f",
               robot_pose.pose.pose.position.x,
               robot_pose.pose.pose.position.y,
               robot_pose.pose.pose.position.z,
               robot_pose.pose.pose.orientation.x,
               robot_pose.pose.pose.orientation.y,
               robot_pose.pose.pose.orientation.z,
               robot_pose.pose.pose.orientation.w);

    }

    found=service_name.find("GetArmPoseGoal");
    if (found!=std::string::npos)
    {

      geometry_msgs::PoseWithCovarianceStamped robot_pose;

      robot_pose.pose.pose.position.x = loadPcd.origin_.x();
      robot_pose.pose.pose.position.y = loadPcd.origin_.y();
      robot_pose.pose.pose.position.z = loadPcd.origin_.z();

      robot_pose.pose.pose.orientation.x = loadPcd.orientation_.x();
      robot_pose.pose.pose.orientation.y = loadPcd.orientation_.y();
      robot_pose.pose.pose.orientation.z = loadPcd.orientation_.z();
      robot_pose.pose.pose.orientation.w = loadPcd.orientation_.w();


      //Call service to extract the objects
      nbv_utilities::SendCloud::Request request;
      nbv_utilities::SendCloud::Response response;

      request.cloud = loadPcd.cloud_;
      request.robot_pose = robot_pose;

      if (!ros::service::call(service_name, request, response)){
        ROS_ERROR("LoadPcd: error sending point cloud to extractObjects");
        return 0;
      }else{
        if(response.result == response.SUCCEEDED)
          ROS_INFO("LoadPcd : send cloud status: SUCCEEDED \n");
        else if(response.result == response.DONE)
          ROS_INFO("LoadPcd : send cloud status: DONE \n");
        else if(response.result == response.FAILED)
          ROS_INFO("LoadPcd : send cloud status: FAILED \n");
      }


    }

    found=service_name.find("KinectCloud");
    if (found!=std::string::npos)
    {

      geometry_msgs::PoseWithCovarianceStamped robot_pose;

      robot_pose.pose.pose.position.x = loadPcd.origin_.x();
      robot_pose.pose.pose.position.y = loadPcd.origin_.y();
      robot_pose.pose.pose.position.z = loadPcd.origin_.z();

      robot_pose.pose.pose.orientation.x = loadPcd.orientation_.x();
      robot_pose.pose.pose.orientation.y = loadPcd.orientation_.y();
      robot_pose.pose.pose.orientation.z = loadPcd.orientation_.z();
      robot_pose.pose.pose.orientation.w = loadPcd.orientation_.w();


      //Call service to extract the objects
      nbv_utilities::SendCloud::Request request;
      nbv_utilities::SendCloud::Response response;

      request.cloud = loadPcd.cloud_;
      request.robot_pose = robot_pose;

      if (!ros::service::call(service_name, request, response)){
        ROS_ERROR("LoadPcd: error sending point cloud to extractObjects");
        return 0;
      }else{
        if(response.result == response.SUCCEEDED)
          ROS_INFO("LoadPcd : send cloud status: SUCCEEDED \n");
        else if(response.result == response.DONE)
          ROS_INFO("LoadPcd : send cloud status: DONE \n");
        else if(response.result == response.FAILED)
          ROS_INFO("LoadPcd : send cloud status: FAILED \n");
      }


    }

    ros::spinOnce();

  }else{
    ROS_ERROR("ENTER FILE NAME AND SERVICE NAME");
    return 0;
  }




  return 0;
}
