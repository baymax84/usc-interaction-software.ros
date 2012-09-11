#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "bandit_msgs/JointArray.h"
#include "bandit_msgs/Params.h"

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Slider.H>
#include <stdio.h>

#define DTOR( a ) ( ( a ) * M_PI / 180.0f )

ros::Publisher joint_publisher;
ros::Publisher joint_state_publisher;

std::vector<std::string> joint_names;

bandit_msgs::Params::Response res;

void slider_cb( Fl_Widget* o, void* )
{
  const char* label = o->label();
  int num = 0;

  for( int i = 0; i < res.name.size(); i++ )
  {
    if( res.name[i] == label ) num = i;
  }
  
  Fl_Valuator* oo = (Fl_Valuator*) o;

/*
	sensor_msgs::JointState js;
	js.header.frame_id="/world";
	js.name.push_back(joint_names[num] + "_joint");
	js.position.push_back(DTOR(oo->value()));
	js.velocity.push_back(0.0);
	js.effort.push_back(0);
  printf( "setting %d: %s\n", num, js.name[0].c_str() );
	joint_state_publisher.publish(js);
	printf("@"); fflush(stdout);
*/

  bandit_msgs::JointArray ja;
  bandit_msgs::Joint j;
  j.id = num;
  j.angle = DTOR( oo->value() );
  ja.joints.push_back(j);
  joint_publisher.publish( ja );
}

int main( int argc, char* argv[] )
{
  ros::init(argc,argv,"bandit_mover");
  ros::NodeHandle n;
  joint_publisher = n.advertise<bandit_msgs::JointArray>("joint_cmd",1000);
	//joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1000);


	joint_names.push_back("bandit_head_pan");
	joint_names.push_back("bandit_head_tilt");

  ros::Rate loop_rate(10);

  bandit_msgs::Params::Request req;


  // wait until Params is received
  while( n.ok() )
  {
    if( ros::service::call("params", req,res ) )
    {
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  bandit_msgs::JointArray joint_array;
  bandit_msgs::Joint j;

  int slider_w = 400;
  int slider_h = 20;
  int padding = 20;
  int num_sliders = res.name.size();

  printf( "size: %d\n", res.name.size() );

  int win_w = padding*2+slider_w;
  int win_h = padding*(1+num_sliders)+num_sliders*slider_h;

  Fl_Window win( win_w, win_h, "Bandit Mover" );
  win.begin();
  Fl_Value_Slider* sliders[19];

  char names[19][256];

  for( int i = 0; i < num_sliders; i++ )
  {
    printf( "adding slider[%d] %s, (%f,%f)\n", res.id[i], res.name[i].c_str(), res.min[i], res.max[i] );

    sliders[i] = new Fl_Value_Slider( padding, (i+1)*padding+i*slider_h,slider_w, slider_h, res.name[i].c_str() );
    sliders[i]->type(FL_HORIZONTAL);
    sliders[i]->bounds(res.min[i], res.max[i] );
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
