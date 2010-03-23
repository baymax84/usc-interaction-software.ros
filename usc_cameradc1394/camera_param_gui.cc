#include "ros/ros.h"
#include "usc_cameradc1394/FeatureValue.h"
#include "usc_cameradc1394/Params.h"
#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Slider.H>
#include <stdio.h>

ros::Publisher param_publisher;
usc_cameradc1394::Params params;
bool received_params;

void slider_cb( Fl_Widget* o, void* )
{

  const char* label = o->label();
  int num = 0;
  for( unsigned int i = 0; i < params.params.size(); i++ )
  {
    if( !strcmp( label, params.params[i].name.c_str() ) )
    {
      printf( "matched: %s\n", params.params[i].name.c_str() );
      num = params.params[i].id;
      break;
    }
  }
  
  Fl_Valuator* oo = (Fl_Valuator*) o;

  if( num != 0 )
  {
    printf( "setting %d(%s): %d\n", num, o->label(),  (int) oo->value() );
    usc_cameradc1394::FeatureValue v;
    v.id = num;
    v.value = (int) oo->value();
    param_publisher.publish( v );
  }
}

void params_cb( const usc_cameradc1394::ParamsConstPtr &msg )
{ 
  ROS_INFO( "received camera params" );
  params = *msg;
  received_params = true;
}

int main( int argc, char* argv[] )
{
  ros::init(argc,argv,"camera_param_gui");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe( "camera_params", 1, params_cb );
  param_publisher = n.advertise<usc_cameradc1394::FeatureValue>("set_param",1000);

  ros::Rate loop_rate(10);

  int slider_w = 400;
  int slider_h = 20;
  int padding = 20;
  int num_sliders = 0;
  received_params = false;

  while (!received_params && n.ok() )
  {
    printf( "waiting for camera_params message...\n" );
    ros::spinOnce();
    loop_rate.sleep();
  }

  sub.shutdown();

  num_sliders = params.params.size();

  int win_w = padding*2+slider_w;
  int win_h = padding*(1+num_sliders)+num_sliders*slider_h;

  Fl_Window win( win_w, win_h, "Camera Params" );
  win.begin();
  Fl_Value_Slider** sliders = new Fl_Value_Slider*[num_sliders];

  for( int i = 0; i < num_sliders; i++ )
  {
    sliders[i] = new Fl_Value_Slider( padding, (i+1)*padding+i*slider_h,slider_w, slider_h, params.params[i].name.c_str() );
    sliders[i]->type(FL_HORIZONTAL);
    sliders[i]->step(1.0);
    sliders[i]->bounds(params.params[i].min, params.params[i].max );
    sliders[i]->value(params.params[i].value);
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


