#ifndef POSE_FROM_FILE_H_
#define POSE_FROM_FILE_H_

#include <pose_model/SimonPose.h>
#include <string>
#include <vector>

pose_model::SimonPose load_pose_from_file( const char* filename  );
std::vector<pose_model::SimonPose> read_poses_from_yaml( std::string filename, std::string prefix );

#endif

