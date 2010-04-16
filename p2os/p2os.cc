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

#include "ros/ros.h"
#include "p2os.h"

void
P2OSNode::StandardSIPPutData(ros::Time ts)
{

  p2os_data.position.header.stamp = ts;
  pose_pub.publish( p2os_data.position );
  batt_pub.publish( p2os_data.batt );
  mstate_pub.publish( p2os_data.motors );

  // put sonar data
  p2os_data.sonar.header.stamp = ts;
  sonar_pub_.publish( p2os_data.sonar );

  // put aio data
  aio_pub_.publish( p2os_data.aio);
  // put dio data
  aio_pub_.publish( p2os_data.dio);

  // put gripper and lift data
  grip_state_pub_.publish( p2os_data.gripper );
  ptz_state_pub_.publish( ptz_.getCurrentState() );

  // put bumper data
  // put compass data

}

/* send the packet, then receive and parse an SIP */
int
P2OSNode::SendReceive(P2OSPacket* pkt, bool publish_data)
{
  P2OSPacket packet;

  if((this->psos_fd >= 0) && this->sippacket)
  {
    if(pkt)
      pkt->Send(this->psos_fd);

    /* receive a packet */
    pthread_testcancel();
    if(packet.Receive(this->psos_fd))
    {
      ROS_ERROR("RunPsosThread(): Receive errored");
      pthread_exit(NULL);
    }

    if(packet.packet[0] == 0xFA && packet.packet[1] == 0xFB &&
       (packet.packet[3] == 0x30 || packet.packet[3] == 0x31 ||
       packet.packet[3] == 0x32 || packet.packet[3] == 0x33 ||
       packet.packet[3] == 0x34))
    {

      /* It is a server packet, so process it */
      this->sippacket->ParseStandard( &packet.packet[3] );
      this->sippacket->FillStandard(&(this->p2os_data));

      if(publish_data)
        this->StandardSIPPutData(packet.timestamp);
    }
    else if(packet.packet[0] == 0xFA && packet.packet[1] == 0xFB &&
            packet.packet[3] == SERAUX)
    {
      // This is an AUX serial packet
      if(ptz_.isOn())
      {
        int len = packet.packet[2] - 3;
        if (ptz_.cb_.gotPacket())
        {
          ROS_ERROR("PTZ got a message, but alread has the complete packet.");
        }
        else
        {
          for (int i=4; i < 4+len; ++i)
          {
            ptz_.cb_.putOnBuf(packet.packet[i]);
          }
        }
      }
    }
    else
    {
      ROS_ERROR("Received other packet!");
      packet.PrintHex();
    }
  }

  return(0);
}

void
P2OSNode::ResetRawPositions()
{
  P2OSPacket pkt;
  unsigned char p2oscommand[4];

  if(this->sippacket)
  {
    this->sippacket->rawxpos = 0;
    this->sippacket->rawypos = 0;
    this->sippacket->xpos = 0;
    this->sippacket->ypos = 0;
    p2oscommand[0] = SETO;
    p2oscommand[1] = ARGINT;
    pkt.Build(p2oscommand, 2);
    this->SendReceive(&pkt,false);
  }
}

/* toggle sonars on/off, according to val */
void
P2OSNode::ToggleSonarPower(unsigned char val)
{
  unsigned char command[4];
  P2OSPacket packet;

  command[0] = SONAR;
  command[1] = ARGINT;
  command[2] = val;
  command[3] = 0;
  packet.Build(command, 4);
  SendReceive(&packet,false);
}

/* toggle motors on/off, according to val */
void
P2OSNode::ToggleMotorPower(unsigned char val)
{
  unsigned char command[4];
  P2OSPacket packet;
  ROS_INFO( "motor state: %d\n", p2os_data.motors.state );
  p2os_data.motors.state = (int) val;
  command[0] = ENABLE;
  command[1] = ARGINT;
  command[2] = val;
  command[3] = 0;
  packet.Build(command, 4);
  SendReceive(&packet,false);
}

/////////////////////////////////////////////////////
//  Actarray stuff
/////////////////////////////////////////////////////

// Ticks to degrees from the ARIA software
inline double P2OSNode::TicksToDegrees (int joint, unsigned char ticks)
{
  if ((joint < 0) || (joint >= sippacket->armNumJoints))
    return 0;

  double result;
  int pos = ticks - sippacket->armJoints[joint].centre;
  result = 90.0 / static_cast<double> (sippacket->armJoints[joint].ticksPer90);
  result = result * pos;
  if ((joint >= 0) && (joint <= 2))
    result = -result;

  return result;
}

// Degrees to ticks from the ARIA software
inline unsigned char P2OSNode::DegreesToTicks (int joint, double degrees)
{
  double val;

  if ((joint < 0) || (joint >= sippacket->armNumJoints))
    return 0;

  val = static_cast<double> (sippacket->armJoints[joint].ticksPer90) * degrees / 90.0;
  val = round (val);
  if ((joint >= 0) && (joint <= 2))
    val = -val;
  val += sippacket->armJoints[joint].centre;

  if (val < sippacket->armJoints[joint].min)
    return sippacket->armJoints[joint].min;
  else if (val > sippacket->armJoints[joint].max)
    return sippacket->armJoints[joint].max;
  else
    return static_cast<int> (round (val));
}

inline double P2OSNode::TicksToRadians (int joint, unsigned char ticks)
{
  double result = DTOR (TicksToDegrees (joint, ticks));
  return result;
}

inline unsigned char P2OSNode::RadiansToTicks (int joint, double rads)
{
  unsigned char result = static_cast<unsigned char> (DegreesToTicks (joint, RTOD (rads)));
  return result;
}

inline double P2OSNode::RadsPerSectoSecsPerTick (int joint, double speed)
{
  double degs = RTOD (speed);
  double ticksPerDeg = static_cast<double> (sippacket->armJoints[joint].ticksPer90) / 90.0f;
  double ticksPerSec = degs * ticksPerDeg;
  double secsPerTick = 1000.0f / ticksPerSec;

  if (secsPerTick > 127)
    return 127;
  else if (secsPerTick < 1)
    return 1;
  return secsPerTick;
}

inline double P2OSNode::SecsPerTicktoRadsPerSec (int joint, double msecs)
{
  double ticksPerSec = 1.0 / (static_cast<double> (msecs) / 1000.0);
  double ticksPerDeg = static_cast<double> (sippacket->armJoints[joint].ticksPer90) / 90.0f;
  double degs = ticksPerSec / ticksPerDeg;
  double rads = DTOR (degs);

  return rads;
}

void P2OSNode::SendPulse (void)
{
  unsigned char command;
  P2OSPacket packet;

  command = PULSE;
  packet.Build(&command, 1);
  SendReceive(&packet);
}
