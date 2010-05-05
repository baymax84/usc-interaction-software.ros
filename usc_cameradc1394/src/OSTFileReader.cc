#include <dcam1394/OSTFileReader.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

bool skipTo( const char * txt, std::ifstream* infile )
{
  std::string line;
  while( getline(*infile, line, '\n' ) )
  {
    //ROS_INFO("%s", line.c_str() );
    if( line == std::string(txt) ) return true;
  }

  return false;
}

std::vector<sensor_msgs::CameraInfo> parseOST( std::string filename )
{
  std::vector<std::string> file;
  std::string line;
  file.clear();
  std::ifstream infile (filename.c_str(), std::ios_base::in);

  std::vector<sensor_msgs::CameraInfo> parsedInfo;
  sensor_msgs::CameraInfo left_msg;
  sensor_msgs::CameraInfo right_msg;
  sensor_msgs::CameraInfo curr;

  bool left_parsed = false;
  bool right_parsed = false;

  while( getline(infile, line, '\n' ) )
  {
    if( !skipTo( "[image]", &infile  )) break;
    if( !skipTo( "width", &infile  )) break;
    if( !getline(infile,line,'\n') ) break;
    sscanf( line.c_str(), "%d", &(curr.width) );
    if( !skipTo( "height", &infile  )) break;
    if( !getline(infile,line,'\n') ) break;
    sscanf( line.c_str(), "%d", &(curr.height) );
    ROS_INFO( "width: %d, height: %d", curr.width, curr.height );
    if( !skipTo( "camera matrix", &infile  )) break;
    //curr.K.reserve(9);
    if( !getline(infile,line,'\n') ) break;
    sscanf( line.c_str(), "%lf %lf %lf", &(curr.K[0]), &(curr.K[1]), &(curr.K[2]) );
    if( !getline(infile,line,'\n') ) break;
    sscanf( line.c_str(), "%lf %lf %lf", &(curr.K[3]), &(curr.K[4]), &(curr.K[5]) );
    if( !getline(infile,line,'\n') ) break;
    sscanf( line.c_str(), "%lf %lf %lf", &(curr.K[6]), &(curr.K[7]), &(curr.K[8]) );

    printf( "K" );
    for( int i = 0; i < 9; i++ )
    {
      if( i % 3 == 0 ) printf( "\n" );
      printf( "%0.3f ", curr.K[i] );
    }
    printf( "\n" );

    if( !skipTo( "distortion", &infile  )) break;
    if( !getline(infile,line,'\n') ) break;
    sscanf( line.c_str(), "%lf %lf %lf %lf %lf", &(curr.D[0]), &(curr.D[1]), &(curr.D[2]), &(curr.D[3]), &(curr.D[4]) );
    printf( "D: " );
    for( int i = 0; i < 5; i++ )
    {
      printf( "%0.3f ", curr.D[i] );
    }
    printf( "\n" );
 
    if( !skipTo( "rectification", &infile  )) break;
    if( !getline(infile,line,'\n') ) break;
    sscanf( line.c_str(), "%lf %lf %lf", &(curr.R[0]), &(curr.R[1]), &(curr.R[2]) );
    if( !getline(infile,line,'\n') ) break;
    sscanf( line.c_str(), "%lf %lf %lf", &(curr.R[3]), &(curr.R[4]), &(curr.R[5]) );
    if( !getline(infile,line,'\n') ) break;
    sscanf( line.c_str(), "%lf %lf %lf", &(curr.R[6]), &(curr.R[7]), &(curr.R[8]) );

    printf( "R" );
    for( int i = 0; i < 9; i++ )
    {
      if( i % 3 == 0 ) printf( "\n" );
      printf( "%0.3f ", curr.R[i] );
    }
    printf( "\n" );

    if( !skipTo( "projection", &infile  )) break;
    if( !getline(infile,line,'\n') ) break;
    sscanf( line.c_str(), "%lf %lf %lf %lf", &(curr.P[0]), &(curr.P[1]), &(curr.P[2]), &(curr.P[3]) );
    if( !getline(infile,line,'\n') ) break;
    sscanf( line.c_str(), "%lf %lf %lf %lf", &(curr.P[4]), &(curr.P[5]), &(curr.P[6]), &(curr.P[7]) );
    if( !getline(infile,line,'\n') ) break;
    sscanf( line.c_str(), "%lf %lf %lf %lf", &(curr.P[8]), &(curr.P[9]), &(curr.P[10]), &(curr.P[11]) );


    printf( "P" );
    for( int i = 0; i < 12; i++ )
    {
      if( i % 4 == 0 ) printf( "\n" );
      printf( "%0.3f ", curr.P[i] );
    }
    printf( "\n" );

    if( !left_parsed )
    {
      left_parsed = true;
      left_msg = curr;
    }
    else
    {
      right_parsed = true;
      right_msg = curr;
    }
 } 
  
  if( left_parsed ) parsedInfo.push_back( left_msg );
  if( right_parsed ) parsedInfo.push_back( right_msg );

  return parsedInfo;
}
