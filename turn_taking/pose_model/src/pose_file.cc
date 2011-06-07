#include <pose_model/SimonPose.h>
#include <opencv/cv.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <cstring>
#include <yaml-cpp/yaml.h>


pose_model::SimonPose load_pose_from_file( const char* filename  )
{
	ros::Time::init();
	FILE* infile = fopen( filename, "r" );
	int num_lines = 0;
	std::vector<float> values;

	while( !feof( infile ) )
	{
		num_lines++;
		for( int i = 0; i < 9; i++ )
		{
			float x = 0;
			fscanf( infile, "%f ", &x );
			values.push_back(x);
			printf( "%0.4f ", x );
		}

		fscanf(infile, "\n" );
		printf( "\n" );
	}

	cv::Mat pose_mat(num_lines,8,CV_64F);
	for( int i = 0; i < num_lines; i++ )
	{
		for( int j = 0; j < 8; j++ )
		{
			pose_mat.at<double>(i,j) = values[i*9+j+1];
			printf( "%0.2f ", pose_mat.at<double>(i,j)  );
		}
		printf( "\n" );
	}

	cv::Mat covar(8,8,CV_64F), mean(8,1,CV_64F);
	cv::calcCovarMatrix(pose_mat, covar, mean, CV_COVAR_NORMAL | CV_COVAR_ROWS );

	for( int i = 0; i < 8; i++ )
	{
		printf( "%0.2f ", mean.at<double>(0,i) );
	}
	printf( "\n" );

	pose_model::SimonPose pose;
	pose.header.stamp = ros::Time::now();

	pose.joint_names.push_back( "left_bicep_forearm_joint" );
	pose.joint_names.push_back( "left_shoulder_mounting_shoulder_joint" );
	pose.joint_names.push_back( "left_torso_shoulder_mounting_joint" );
	pose.joint_names.push_back( "left_shoulder_bicep_joint" );
	pose.joint_names.push_back( "right_bicep_forearm_joint" );
	pose.joint_names.push_back( "right_shoulder_mounting_shoulder_joint" );
	pose.joint_names.push_back( "right_torso_shoulder_mounting_joint" );
	pose.joint_names.push_back( "right_shoulder_bicep_joint" );

	for( int i = 0; i < 8; i++ )
	{
		pose.joint_poses.push_back( mean.at<double>(0,i) );
		for( int j = 0; j < 8; j++ )
		{
			pose.covariance.push_back( covar.at<double>(i,j) );
		}
	}

	return pose;
}

std::vector<pose_model::SimonPose> read_poses_from_yaml( std::string filename )
{
	std::ifstream fin;
	fin.open( filename.c_str() );

	std::vector<pose_model::SimonPose> ret;

	if( fin.fail() )
	{
		ROS_WARN("couldn't find the file: [%s]", filename.c_str() );
		return ret;
	}

	YAML::Node doc;
	YAML::Parser parser(fin);
	parser.GetNextDocument(doc);
	const YAML::Node& y_files = doc[0]["files"];
	
	std::string modelname;
	for( int i = 0; i < y_files.size(); i++ )
	{
		std::string fname;
		y_files[i]["filename"] >> fname;
		ret.push_back(load_pose_from_file( fname.c_str() ));
		ROS_INFO( "adding: [%s] to model", fname.c_str() );
	}

}

int main( int argc, char* argv[] )
{
	read_poses_from_yaml( argv[1] );
	return 0;
}

