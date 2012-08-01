/***************************************************************************
 *  include/quickdev/controllers/reconfigurable_pid.h
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

#ifndef QUICKDEVCPP_QUICKDEV_CONTROLLERS_RECONFIGURABLEPID_H_
#define QUICKDEVCPP_QUICKDEV_CONTROLLERS_RECONFIGURABLEPID_H_

#include <quickdev/controllers/pid.h>
#include <quickdev/reconfigure_policy.h>

#include <quickdev_cpp/PIDConfig.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

typedef quickdev_cpp::PIDConfig _PIDConfig;
typedef ReconfigurePolicy<_PIDConfig> _PIDLiveParams;

class ReconfigurablePIDSettings : public PIDSettings, public _PIDLiveParams
{
public:
    ReconfigurablePIDSettings()
    :
        PIDSettings(),
        _PIDLiveParams( ros::NodeHandle( "~" ) )
    {
        //
    }

    /*ReconfigurablePIDSettings( ReconfigurablePIDSettings & other )
    :
        PIDSettings(),
        _PIDLiveParams( ros::NodeHandle( "~" ) )
    {
        PRINT_INFO( "Copying settings." );
        _PIDLiveParams::getInstance() = other.getPIDLiveParams();

        _PIDLiveParams::registerCallback( quickdev::auto_bind( &ReconfigurablePIDSettings::reconfigureCB, this ) );
        PRINT_INFO( "Done copying settings" );
    }

    _PIDLiveParams & getPIDLiveParams()
    {
        return _PIDLiveParams::getInstance();
    }*/

    ReconfigurablePIDSettings( std::string const & name, std::string const & ns = "pid/" )
    :
        PIDSettings(),
        _PIDLiveParams( ros::NodeHandle( "~" ) )
    {
        _PIDLiveParams::registerCallback( quickdev::auto_bind( &ReconfigurablePIDSettings::reconfigureCB, this ) );
        _PIDLiveParams::init( "reconfigure_namespace_param", ns + name );
    }

    QUICKDEV_DECLARE_RECONFIGURE_CALLBACK( reconfigureCB, _PIDConfig )
    {
        p_ = config.p;
        i_ = config.i;
        d_ = config.d;
        output_min_ = config.output_min;
        output_max_ = config.output_max;
        direction_ = config.output_dir;

        if( settings_changed_callback_ ) settings_changed_callback_();
    }
};

template<unsigned int __Dim__>
class ReconfigurablePID : public PID<__Dim__, ReconfigurablePIDSettings>
{
public:
    template<class... __Args>
    ReconfigurablePID( __Args&&... args )
    :
        PID<__Dim__, ReconfigurablePIDSettings>( std::forward<__Args>( args )... )
    {
        //
    }
};

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_CONTROLLERS_RECONFIGURABLEPID_H_
