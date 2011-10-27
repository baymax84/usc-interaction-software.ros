/***************************************************************************
 *  include/sbl_teleop/sbl_teleop.h
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

#ifndef SBL_TELEOP_SBL_TELEOP_SBL_TELEOP_H_
#define SBL_TELEOP_SBL_TELEOP_SBL_TELEOP_H_

#include <base_libs/node.h>
#include <base_libs/joystick_policy.h>

typedef base_libs::JoystickPolicy _JoystickPolicy;

BASE_LIBS_DECLARE_NODE( SblTeleop, _JoystickPolicy )

BASE_LIBS_DECLARE_NODE_CLASS( SblTeleop )
{
public:
	bool publish_cmd_vel_;
	_JoystickPolicy::_AxisName current_button_;
	
	
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( SblTeleop ),
		publish_cmd_vel_( true ),
		current_button_( "" )
	{
		//
	}
	
	bool tryGetButtonLock( const _JoystickPolicy::_Axis & axis, const _JoystickPolicy::_JoystickMsg::ConstPtr & msg )
	{
		if( axis.getValueAsButton( msg ) > 0 )
		{
			if( current_button_ == "" )
			{
				//PRINT_INFO( "Axis [ %s ] gained lock", axis.name_.c_str() );
				
				current_button_ = axis.name_;
				return true;
			}
		}
		else
		{
			if( current_button_ == axis.name_ )
			{
				//PRINT_INFO( "Axis [ %s ] released lock", axis.name_.c_str() );
				current_button_ = "";
			}
		}
		
		return false;
	}
	
	BASE_LIBS_DECLARE_MESSAGE_CALLBACK( joystickCB, _JoystickPolicy::_JoystickMsg )
	{
		if( JoystickPolicy::isEnabled() )
		{
			const auto cmd_vel_publish_toggle_axis = _JoystickPolicy::getAxis( "cmd_vel_publish_toggle" );
			
			if( tryGetButtonLock( cmd_vel_publish_toggle_axis, msg ) ) publish_cmd_vel_ = !publish_cmd_vel_;
		}
	}
	
	void spinFirst()
	{
		initAll();
	}
	
	void spinOnce()
	{
		// update but don't publish
		_JoystickPolicy::update( false );
		// publish only if we've toggled publishing on
		if( publish_cmd_vel_ ) _JoystickPolicy::publish();
	}
};

#endif // SBL_TELEOP_SBL_TELEOP_SBL_TELEOP_H_
