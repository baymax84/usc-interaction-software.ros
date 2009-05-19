/*
 *  P2OS for ROS
 *  Copyright (C) 2009
 *     David Feil-Seifer, Brian Gerkey, Kasper Stoy, 
 *      Richard Vaughan, & Andrew Howard
 *
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

#ifndef _P2OSDEVICE_H
#define _P2OSDEVICE_H

#include <pthread.h>
#include <sys/time.h>
#include <iostream>
#include <string.h>

#include "packet.h"
#include "robot_params.h"

#include "deprecated_msgs/RobotBase2DOdom.h"
#include "robot_msgs/PoseWithRatesStamped.h"
#include "robot_msgs/PoseDot.h"
#include "robot_msgs/BatteryState.h"
#include "p2os/MotorState.h"

typedef struct ros_p2os_data
{
	robot_msgs::PoseWithRatesStamped  position;
  deprecated_msgs::RobotBase2DOdom  odom;
  robot_msgs::BatteryState          batt;
  p2os::MotorState                  motors;
} ros_p2os_data_t;


// this is here because we need the above typedef's before including it.
#include "sip.h"
#include "kinecalc.h"

class SIP;

// Forward declaration of the KineCalc_Base class declared in kinecalc_base.h
//class KineCalc;


class P2OSNode
{
  public:
    P2OSNode();
    virtual ~P2OSNode();
    
  public:
    int Setup();
    int Shutdown();

    int SendReceive(P2OSPacket* pkt, bool publish_data = true );
    void ResetRawPositions();
    void ToggleSonarPower(unsigned char val);
    void ToggleMotorPower(unsigned char val);
    void StandardSIPPutData(ros::Time ts);

    inline double TicksToDegrees (int joint, unsigned char ticks);
    inline unsigned char DegreesToTicks (int joint, double degrees);
    inline double TicksToRadians (int joint, unsigned char ticks);
    inline unsigned char RadiansToTicks (int joint, double rads);
    inline double RadsPerSectoSecsPerTick (int joint, double speed);
    inline double SecsPerTicktoRadsPerSec (int joint, double secs);

    void SendPulse (void);
    void spin();
    void set_vel();
    void cmdvel_cb();

    void set_motor_state();
    void cmdmotor_state();

  protected:
    SIP* sippacket;
    std::string psos_serial_port;
    std::string psos_tcp_host;
    int         psos_fd;
    bool        psos_use_tcp;
    int         psos_tcp_port;
    bool        vel_dirty, motor_dirty;
    int         param_idx;
    // PID settings
    int rot_kp, rot_kv, rot_ki, trans_kp, trans_kv, trans_ki; 

    int bumpstall; // should we change the bumper-stall behavior?
    int joystick;
    int direct_wheel_vel_control;
    int radio_modemp;

    int motor_max_speed;
    int motor_max_turnspeed;
    short motor_max_trans_accel, motor_max_trans_decel;
    short motor_max_rot_accel, motor_max_rot_decel;
    double pulse; // Pulse time
    double lastPulseTime; // Last time of sending a pulse or command to the robot

  public:
    robot_msgs::PoseDot cmdvel;
    p2os::MotorState    cmdmotor_state_;
	  ros_p2os_data_t p2os_data;

};

#endif
