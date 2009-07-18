#include "ros/ros.h"
#include "bandit_msgs/JointArray.h"

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Slider.H>
#include <stdio.h>

#define DTOR( a ) a * M_PI / 180.0

ros::Publisher joint_publisher;

void slider_cb( Fl_Widget* o, void* )
{
  const char* label = o->label();
  int num = atoi(label);
  
  Fl_Valuator* oo = (Fl_Valuator*) o;

  printf( "setting %d: %f\n", num, oo->value() );

  bandit_msgs::Joint j;
  j.id = num;
  j.angle = DTOR( oo->value() );

  joint_publisher.publish( j );
}

int main( int argc, char* argv[] )
{
  ros::init(argc,argv,"bandit_mover");
  ros::NodeHandle n;
  joint_publisher = n.advertise<bandit_msgs::Joint>("joint_ind",1000);
  ros::Rate loop_rate(10);

  bandit_msgs::JointArray joint_array;
  bandit_msgs::Joint j;

  int slider_w = 400;
  int slider_h = 20;
  int padding = 20;
  int num_sliders = 19;

  int win_w = padding*2+slider_w;
  int win_h = padding*(1+num_sliders)+num_sliders*slider_h;

  Fl_Window win( win_w, win_h, "Bandit Mover" );
  win.begin();
  Fl_Value_Slider* sliders[19];

  char names[19][256];

  for( int i = 0; i < num_sliders; i++ )
  {
    sprintf( names[i], "%d", i );
    sliders[i] = new Fl_Value_Slider( padding, (i+1)*padding+i*slider_h,slider_w, slider_h, names[i] );
    sliders[i]->type(FL_HORIZONTAL);
    sliders[i]->bounds(-180.0, 180.0 );
    sliders[i]->callback( slider_cb );
  }

  win.end();
  win.show();

  while( n.ok() && Fl::check() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
