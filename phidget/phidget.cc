/*
 *  PhidgetIFK for ROS
 *  Copyright (C) 2009
 *     David Feil-Seifer
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */



#include "ros/ros.h"
#include "phidget/DigOut.h"
#include "phidget/DigIn.h"
#include "phidget21.h"

class Phidget {

public:
  Phidget(ros::NodeHandle n );
  ~Phidget();

  void digout_set_cb(const phidget::DigOutConstPtr&);
  bool spin();
  int status;
protected:
  CPhidgetInterfaceKitHandle ifk;
  double samplingrate;
  int phidget_serial;
  int phidget_num_outputs;
  int phidget_num_inputs;
  int* inputs;
  int* outputs;

    ros::NodeHandle n_;
    ros::Publisher digin_pub_;
    ros::Subscriber digout_sub_;
  phidget::DigIn  digin_msg_;

};

Phidget::Phidget( ros::NodeHandle n )
{
  n_ = n;

  n_.param("serial", phidget_serial, -1 );
  n_.param("samplingrate", samplingrate, 20.0 );

  CPhidgetInterfaceKit_create(&ifk);
  CPhidget_open((CPhidgetHandle)ifk, phidget_serial);

  status = -1;
  status = CPhidget_waitForAttachment((CPhidgetHandle)ifk, 1000);

  if( status != 0 )
  {
    printf( "could not connect to phidget... aborting... \n" );
  }
  else
  {
    printf( "connected to phidget...\n" );
  }

  // get phidget info

  phidget_num_outputs = 0;
  phidget_num_inputs = 0;
  CPhidgetInterfaceKit_getNumOutputs(ifk, &phidget_num_outputs);
  CPhidgetInterfaceKit_getNumInputs(ifk, &phidget_num_inputs);

  // TODO: handle analog inputs

  // format data

  outputs = new int[phidget_num_outputs];
  inputs = new int[phidget_num_inputs];

  digin_msg_.inputs.resize(phidget_num_inputs);

  // advertise/subscribe services

  digin_pub_ = n_.advertise<phidget::DigIn>( "digin", 1000 );
  digout_sub_ = n_.subscribe( "digout", 1, (boost::function< void(const phidget::DigOutConstPtr&)>) boost::bind(&Phidget::digout_set_cb, this,_1));

  // TODO: advertise analog service

}


Phidget::~Phidget()
{
  usleep(100000);
  CPhidget_close((CPhidgetHandle)ifk);
  CPhidget_delete((CPhidgetHandle)ifk);
  ifk = 0;
}

void
Phidget::digout_set_cb( const phidget::DigOutConstPtr& digout_msg )
{
  int i = digout_msg->num;
  if( i < phidget_num_outputs )
  {
    int x = 0;
    if( digout_msg->output != 0 ) x = 1;
    CPhidgetInterfaceKit_setOutputState( ifk, i, x );
  }
  else
  {
    ROS_WARN( "phidget output request [%d] exceeded max of %d\n", i, phidget_num_outputs );
  }
}

bool
Phidget::spin()
{
  ros::Time t = ros::Time::now();

  bool dirty = false;
  for( int i = 0; i < phidget_num_inputs; i++ )
  {
    int tmp = 0;
    CPhidgetInterfaceKit_getInputState(ifk,i,&tmp);
    if( tmp != inputs[i] )
    {
      inputs[i] = tmp;
      digin_msg_.inputs[i] = tmp;
      dirty = true;
    }
  }

  if( dirty )
  {
    printf( "input state: " );
    for( int i = 0; i < phidget_num_inputs; i++ )
    {
      printf( "%d", digin_msg_.inputs[i] );
    }
    printf( "\n" );
    digin_msg_.header.stamp = t;
    digin_pub_.publish(digin_msg_);
  }

  // TODO: handle analog inputs

  return true;
}

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "phidget" );

  ros::NodeHandle n( "~" );
    ros::Rate loop_rate(20);
  Phidget p(n);

  if( p.status != 0 )
  {
    return -1;
  }

  while( n.ok() && p.spin() )
  {
        ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
