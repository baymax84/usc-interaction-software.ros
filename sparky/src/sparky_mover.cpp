// preprocessor directives
#include <ros/ros.h>
#include <joint_msgs/JointArray.h>
#include <joint_msgs/Params.h>
#include <sensor_msgs/JointState.h>

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Slider.H>
#include <stdio.h>



// angle (degrees and radians) definitions
#define DTOR(deg) ((deg) * M_PI / 180.0f)



// global variables
ros::Publisher               g_joint_pub;
joint_msgs::Params::Response g_params_res;



//
void sliderCallback(Fl_Widget* o, void*)
{
  const char* label = o->label();
  printf("^");
  fflush(stdout);

  int id = -1;
  for (int i = 0, n = g_params_res.name.size(); i < n; ++i)
    if (g_params_res.name[i] == label) id = i;

  printf("$");
  fflush(stdout);

  Fl_Valuator* oo = (Fl_Valuator*) o;
	
  printf("!");
  fflush(stdout);

  joint_msgs::Joint joint;
  joint.id     = id;
  joint.angle  = oo->value();
  joint.active = true;

  g_joint_pub.publish(joint);
} // sliderCallback(Fl_Widget*, void*)



//
int main(int argc, char** argv)
{

  // initialize ROS node
  ros::init(argc, argv, "sparky_mover");
  ros::NodeHandle nh;

  // set up ROS publishers
  g_joint_pub = nh.advertise<joint_msgs::Joint>("joint_cmd", 1000);

  // set up ROS subscribers
  //ros::Subscriber joints_sub = nh.subscribe(
  //                               "joints_cmd", 1, jointArrayCallback);
  //ros::Subscriber joint_sub  = nh.subscribe(
  //                               "joint_cmd",  1, jointCallback);

  // set up ROS services
  joint_msgs::Params::Request params_req;
  ros::Rate loop_rate(10);
  while (nh.ok())
  {
    if (ros::service::call("sparky_params", params_req, g_params_res)) break;
    printf(".");
    fflush(stdout);
    ros::spinOnce();
  }

  // set up sliders
  int       slider_w  = 400;
  int       slider_h  = 20;
  int       padding   = 20;
  const int n_sliders = g_params_res.name.size();

  // set up window
  int win_w = padding * 2 + slider_w;
  int win_h = padding * (1 + n_sliders) + n_sliders * slider_h;
  Fl_Window win(win_w, win_h, "Sparky Mover");
  win.begin();

  // initialize sliders
  Fl_Value_Slider* sliders[n_sliders];
  for (int i = 0; i < n_sliders; i++)
  {
    printf("Adding slider[%d] %s, (%f, %f)\n",
           g_params_res.id[i],
           g_params_res.name[i].c_str(),
           g_params_res.min[i],
           g_params_res.max[i]);
    sliders[i] = new Fl_Value_Slider(padding,
                                     (i + 1) * padding + i * slider_h,
                                     slider_w,
                                     slider_h,
                                     g_params_res.name[i].c_str());
    sliders[i]->type(FL_HORIZONTAL);
    sliders[i]->bounds(g_params_res.min[i], g_params_res.max[i]);
    sliders[i]->callback(sliderCallback);
  }

  // display window
  win.end();
  win.show();

  //
  loop_rate = ros::Rate(50.0f);
  while((nh.ok()) && (Fl::check()))
  {
    printf("+");
    fflush(stdout);
    ros::spinOnce();
    printf("-");
    fflush(stdout);
    loop_rate.sleep();
  }

  // exit program
  return 0;
} // main(int, char**)
