/***************************************************************************
 *  nodes/pid.cpp
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of usc-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#include <quickdev/controllers/reconfigurable_pid.h>
#include <ros/rate.h>

class PIDNode
{
public:
    typedef quickdev::ReconfigurablePID<6> _Pid6D;
    typedef quickdev::ReconfigurablePID<3> _Pid3D;
    typedef quickdev::ReconfigurablePID<1> _Pid;

    _Pid6D pid6d_;
    _Pid3D pid3d_;
    _Pid pid_;
    ros::Rate loop_rate_;

    PIDNode( ros::NodeHandle & nh )
    :
        loop_rate_( 10 )
    {
        auto loop_rate = ros::ParamReader<double, 1>::readParam( nh, "loop_rate", 10 );
        loop_rate_ = ros::Rate( loop_rate );

        pid_.applySettings( quickdev::make_shared( new _Pid::_Settings( "motor" ) ) );

        pid3d_.applySettings(
            quickdev::make_shared( new _Pid3D::_Settings( "field/x" ) ),
            quickdev::make_shared( new _Pid3D::_Settings( "field/y" ) ),
            quickdev::make_shared( new _Pid3D::_Settings( "field/z" ) )
        );

        pid6d_.applySettings(
            quickdev::make_shared( new _Pid6D::_Settings( "linear/x" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "linear/y" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "linear/z" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "angular/x" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "angular/y" ) ),
            quickdev::make_shared( new _Pid6D::_Settings( "angular/z" ) )
        );
    }

    QUICKDEV_SPIN_ONCE()
    {

    }

    void spin()
    {
        while( ros::ok() )
        {
            spinOnce();
            ros::spinOnce();
            loop_rate_.sleep();
        }
    }
};

QUICKDEV_INST_NODE( PIDNode, "pid" )
