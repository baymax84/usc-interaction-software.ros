#include "ros/ros.h"
#include "bandit_msgs/JointArray.h"

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Slider.H>
#include <stdio.h>

#define DTOR( a ) a * M_PI / 180.0

void slider_cb( Fl_Widget* o, void* )
{
  
}

int main( int argc, char* argv[] )
{
  ros::init(argc,argv,"bandit_mover");
  ros::NodeHandle n;
  ros::Publisher joint_publisher = n.advertise<bandit_msgs::JointArray>("joint_ind",1000);
  ros::Rate loop_rate(10);

  bandit_msgs::JointArray joint_array;
  bandit_msgs::Joint j;

  int slider_w = 200;
  int slider_h = 50;
  int padding = 10;
  int num_sliders = 19;

  int win_w = padding*2+slider_w;
  int win_h = padding*(1+num_sliders)+num_sliders*slider_h;

  Fl_Window win( win_w, win_h, "Bandit Mover" );
  win.begin();
  Fl_Value_Slider* sliders[19];

  for( int i = 0; i < num_sliders; i++ )
  {
    sliders[i] = new Fl_Value_Slider( padding, (i+1)*padding+i*slider_h,slider_w, slider_h, "slider" );
    sliders[i]->callback( slider_cb );
  }

  win.end();
  win.show();

  while( n.ok() && Fl::check() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  /*
  j.id = 0;
  j.angle = 0;

  while( n.ok() )
  {
    j.angle = j.angle + DTOR(1);
    if( j.angle > DTOR(30) ) j.angle = 0;
    ROS_INFO( "setting joint [%d] to angle [%0.2f]", j.id, j.angle );

    //joint_array.joints.clear();
    //joint_array.joints.push_back(j);

    joint_publisher.publish( j );

    ros::spinOnce();
    loop_rate.sleep();
  }
  */
  return 0;
}
