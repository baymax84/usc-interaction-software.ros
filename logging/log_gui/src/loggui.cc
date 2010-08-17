#include <iostream>
#include "ros/ros.h"
#include "log_msgs/LogState.h"
#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Light_Button.H>

ros::Publisher log_pub;

void start_cb(Fl_Widget* o, void* )
{
  printf( "start pushed\n" );
  log_msgs::LogState l;
  l.state = 0;
  log_pub.publish( l );
}

void stop_cb(Fl_Widget*o, void* )
{
  printf( "stop pushed\n" );
  log_msgs::LogState l;
  l.state = 1;
  log_pub.publish( l );
}

int main( int argc, char* argv[] )
{
  ros::init(argc,argv, "log_gui");
  ros::NodeHandle n;
  int but_w = 100;
  int but_h = 100;
  int padding = 10;
  int win_w = padding*4+but_w*3;
  int win_h = 2*but_h+padding*3;

  Fl_Window win(win_w, win_h,"LogGui");
  win.begin();
  Fl_Button reenable( padding, win_h-2*padding-2*but_h, win_w-2*padding, but_h, "Start Log" );

  Fl_Button stop(padding, win_h-padding-but_h, win_w-2*padding, but_h, "Stop Log");
  win.end();

  reenable.callback( start_cb );
  stop.callback( stop_cb );
  win.show();

  log_pub = n.advertise<log_msgs::LogState>("/b3ia/logstate",1000);

  ros::Rate loop(10);

  while( n.ok() && Fl::check() )
  {
    printf("."); fflush(stdout);
    ros::spinOnce();
    loop.sleep();
  }
  
}
