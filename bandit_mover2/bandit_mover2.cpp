#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "bandit_msgs/JointArray.h"
#include "bandit_msgs/Params.h"

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Value_Slider.H>
#include <stdio.h>
#include <sys/time.h>

#define DTOR( a ) ( ( a ) * M_PI / 180.0f )

double get_time()
{
  struct timeval tp;
  gettimeofday(&tp,0);
  return ((double)tp.tv_sec + (double)tp.tv_usec/1e6);
}

double goal_array[20];
double current_array[20];
//Fl_Value_Slider* p[20];
Fl_Check_Button* p;

ros::Publisher joint_publisher;
ros::Publisher joint_state_publisher;

std::vector<std::string> joint_names;

bandit_msgs::Params::Response res;

void moveToPosition(){

	bandit_msgs::JointArray jarray;
	printf("I'm moving!!!\n");
	int num_moving = 1;
	double cur_time, diff_time, g_time;
	double last_time = ros::Time::now().toSec();
	double vel;
	double max_vel = 40; // Degrees/second
        g_time = get_time();
	ros::Rate loop_rate(30);
	ros::NodeHandle n;

	while(n.ok() && num_moving > 0){
		num_moving = 0;
		cur_time = ros::Time::now().toSec();
		g_time = get_time();
		diff_time = (g_time - last_time);
		last_time = g_time;

		vel = diff_time * max_vel;
		printf("Time: %f  Diff: %f  Max: %f\n",g_time,diff_time,vel);
		//printf("Time: %f  Time2: %f Diff: %f\n",cur_time,g_time,fabs(cur_time-g_time));

		for (int q = 0; q < 20; q++){

			double c = current_array[q];
			double g = goal_array[q];
			double dist;

			if(fabs(g-c) > 1e-6){
				num_moving++;
	
				dist = fabs(g - c);
				if ( g > c){
					if ( dist > vel)
						c+=vel;
					else
						c+=dist;
				}
				else{
					if ( dist > vel)
						c-=vel;
					else
						c-=dist;
				}
				current_array[q] = c;
				bandit_msgs::Joint j;
				j.id = q;
				j.angle = DTOR(c);
				printf("move joint: %d ang: %f\n",q,c);
				jarray.joints.push_back(j);
				//joint_publisher.publish( j );
			}
		}	
		joint_publisher.publish( jarray );
		ros::spinOnce();
		loop_rate.sleep();
	}
	printf("Done Moving!\n");
}

void button_cb( Fl_Widget* obj , void* )
{
	ros::NodeHandle n;
	joint_publisher = n.advertise<bandit_msgs::JointArray>("joints",5);
	moveToPosition();
}


void slider_cb( Fl_Widget* o, void* )
{
//  bandit_msgs::JointArray jarray;
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
//#if 0
if (p->value(0)){
	bandit_msgs::JointArray jarray;
	bandit_msgs::Joint j;
	j.id = num;
//	j.angle = DTOR( oo->value() );
	if ( 0/*(num == 7) || (num == 8) || (num > 13)*/ )
		j.angle = oo->value();
	else
		j.angle = DTOR( oo->value() );

	jarray.joints.push_back(j);
	joint_publisher.publish( jarray );
}


//#endif

	goal_array[num] = oo->value();
}

void reset_cb( Fl_Widget* o, void* )
{
	int num = 0;
	int stopper = 0;
	ros::Rate loop_rate(30);
	ros::NodeHandle n;

	while(stopper != 1){
		bandit_msgs::JointArray jrarray;
		bandit_msgs::Joint jr;
		printf("Resetting Positions of JointID: %d!\n", num);
		for (num = 0; num < 20; num++){	
			double c = current_array[num];
			goal_array[num] = 0;

				jr.id = num;
				jr.angle = 0; 
			
			current_array[num] = c;
			jrarray.joints.push_back(jr);
			
			//p[num]->value(0);
		}
		joint_publisher.publish( jrarray );
		ros::spinOnce();
		loop_rate.sleep();
		stopper++;
	}
}

void check_cb( Fl_Widget* o, void* )
{



}

int main( int argc, char* argv[] )
{
	ros::init(argc,argv,"bandit_mover");
	ros::NodeHandle n;
	//joint_publisher = n.advertise<bandit_msgs::JointArray>("joint_ind",1000);
	joint_publisher = n.advertise<bandit_msgs::JointArray>("joints",5);
	//joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1000);


	joint_names.push_back("bandit_head_pan");
	joint_names.push_back("bandit_head_tilt");

	ros::Rate loop_rate(30);

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

	for (int asdf = 0; asdf < 20; asdf++)
		current_array[asdf] = 0;

 
#if 0
int k = 0;
while(n.ok()){
ros::spinOnce();
  j.id = 0;
  //j.angle = DTOR( oo->value() );
  //if ( 0/*(num == 7) || (num == 8) || (num > 13)*/ )
    j.angle = DTOR(0);
  //else
   // j.angle = DTOR( oo->value() );

  joint_publisher.publish( j );

ros::spinOnce();
loop_rate.sleep();
k++;
if(k >1)
	break;	
}

getchar();

  j.id = 0;
  //j.angle = DTOR( oo->value() );
  //if ( 0/*(num == 7) || (num == 8) || (num > 13)*/ )
    j.angle = DTOR(-90);
  //else
   // j.angle = DTOR( oo->value() );

  joint_publisher.publish( j );
ros::spinOnce();
loop_rate.sleep();

getchar();

  j.id = 0;
  //j.angle = DTOR( oo->value() );
  //if ( 0/*(num == 7) || (num == 8) || (num > 13)*/ )
    j.angle = DTOR(90);
  //else
   // j.angle = DTOR( oo->value() );

  joint_publisher.publish( j );
ros::spinOnce();
loop_rate.sleep();


getchar();

double vel;
double max_vel = 40; // Degrees/second
double c = 90;
double g = -90;
double dist;
double cur_time, diff_time, g_time;
double last_time = ros::Time::now().toSec();

while(fabs(g-c) > 1e-6){
cur_time = ros::Time::now().toSec();
g_time = get_time();
diff_time = (g_time - last_time);
last_time = g_time;
vel = diff_time * max_vel;

printf("Time: %f  Diff: %f  Max: %f\n",g_time,diff_time,vel);
//printf("Time: %f  Time2: %f Diff: %f\n",cur_time,g_time,fabs(cur_time-g_time));

dist = fabs(g - c);
if ( g > c){
	if ( dist > vel)
		c+=vel;
	else
		c+=dist;
}
else{
	if ( dist > vel)
		c-=vel;
	else
		c-=dist;
}
  j.id = 0;
  j.angle = DTOR(c);

  joint_publisher.publish( j );
  ros::spinOnce();
  loop_rate.sleep();

}

printf("We're here!!!\n");

getchar();
#endif
#if 1
	printf( "size: %d\n", res.name.size() );

	int win_w = padding*2+slider_w;
	int win_h = padding*(4+num_sliders)+(num_sliders+1)*slider_h;

	Fl_Window win( win_w, win_h, "Bandit Mover" );
	win.begin();
	Fl_Value_Slider* sliders[19];

//	char names[19][256];
	int i=0;
	for( i = 0; i < num_sliders; i++ )
	{
		printf( "adding slider[%d] %s, (%f,%f)\n", res.id[i], res.name[i].c_str(), res.min[i], res.max[i] );

		sliders[i] = new Fl_Value_Slider( padding, (i+1)*padding+i*slider_h,slider_w, slider_h, res.name[i].c_str() );
		sliders[i]->type(FL_HORIZONTAL);
		sliders[i]->bounds(res.min[i], res.max[i] );
		sliders[i]->callback( slider_cb );
		//p[i] = sliders[i];
	}

	Fl_Button* butn = new Fl_Button( padding, (i+1)*padding+i*slider_h,slider_w, slider_h, "Go To Position");
		butn->callback( ( Fl_Callback* ) button_cb );

	Fl_Button* resetbutn = new Fl_Button( padding, (i+2)*padding+i*slider_h,slider_w, slider_h, "Reset");
		resetbutn->callback( ( Fl_Callback* ) reset_cb );

	Fl_Check_Button* checkbutn = new Fl_Check_Button( padding, (i+3)*padding+i*slider_h,slider_w, slider_h, "Enable");
		checkbutn->callback( ( Fl_Callback* ) check_cb );
	p = checkbutn;

	win.end();
	win.show();
	while( n.ok() && Fl::check() )
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

#endif

	return 0;
}
