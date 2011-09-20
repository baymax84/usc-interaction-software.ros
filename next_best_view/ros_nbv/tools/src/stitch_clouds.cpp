/*
 * stitch_clouds.cpp
 *
 *  Created on: Aug 15, 2011
 *      Author: potthast
 */

#include <tools/stitch_clouds.h>

StitchClouds::StitchClouds(StitchClouds::command_arg_t &command_arg)
{
  PointCloudRGB cloud;
  PointCloudRGB combined_cloud;

  for(unsigned int i=0; i<command_arg.i_name.size(); i++)
  {
    if (command_arg.i_name[i] == "" || pcl::io::loadPCDFile (command_arg.i_name[i], cloud) == -1)
      std::cout << "ERROR" << std::endl;

    if(i<1)
      combined_cloud = cloud;
    else
      combined_cloud += cloud;
  }

  if(command_arg.downsampling)
  {
    float leaf_size = command_arg.leaf_size;
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (combined_cloud.makeShared());
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);
    sor.filter (combined_cloud);
  }

  pcl::io::savePCDFileASCII (command_arg.o_name , combined_cloud);

}

int main(int argc, char** argv)
{

  StitchClouds::command_arg_t command_arg;
  command_arg.downsampling = false;

  if(argc < 3)
  {
    ROS_ERROR ("No input file specified");
    ROS_ERROR ("e.g.  stitch_clouds -ds 0.01 -o test.pcd -i input_1.pcd input_2.pcd");
    return (-1);
  }

  if(strcmp(argv[1], "help") == 0){
    ROS_ERROR ("e.g.  stitch_clouds -ds 0.01 -o test.pcd -i input_1.pcd input_2.pcd");
  }

  std::vector<std::string> i_arg;
  for(int i=1; i<argc; i++)
  {
    i_arg.push_back(argv[i]);
  }

  for(unsigned int i=0; i<i_arg.size(); i++)
  {
    if(strcmp(i_arg[i].c_str(), "-ds") == 0)
    {
      command_arg.downsampling = true;
      float leaf_size = boost::lexical_cast<float>( i_arg[i+1] );
      command_arg.leaf_size = leaf_size;
      i_arg.erase(i_arg.begin()+i, i_arg.begin()+i+2 );
      i=0;
    }

    if(strcmp(i_arg[i].c_str(), "-o") == 0)
    {
      command_arg.o_name = i_arg[i+1];
      i_arg.erase(i_arg.begin()+i, i_arg.begin()+i+2 );
      i=0;
    }

    if(strcmp(i_arg[i].c_str(), "-i") == 0)
    {
      int count=0;
      for(unsigned int ii=1; ii<i_arg.size(); ii++)
      {
        std::string first_character = i_arg[ii].substr(0,1);
        if(strcmp(first_character.c_str(), "-") == 0)
          break;
        else
        {
          command_arg.i_name.push_back(i_arg[ii]);
          count++;
        }
      }
      i_arg.erase(i_arg.begin()+i, i_arg.begin()+i+1+count );
    }
  }

  if(command_arg.downsampling)
    std::cout << "downsampling: True ; leaf size: " << command_arg.leaf_size << std::endl;
  if(!command_arg.downsampling)
    std::cout << "downsampling: False" << std::endl;
  std::cout << "input: ";
  for(unsigned int i=0; i<command_arg.i_name.size(); i++)
  {
    std::cout << command_arg.i_name[i] << " ";
  }
  std::cout << std::endl;
  std::cout << "output:" << command_arg.o_name << std::endl;

  StitchClouds stitchClouds(command_arg);


  return 0;
}
