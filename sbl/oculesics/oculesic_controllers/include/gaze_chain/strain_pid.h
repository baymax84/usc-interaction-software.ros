/*******************************************************************************
 *
 *      strain_pid
 * 
 *      Copyright (c) 2011, edward
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "interaction-ros-pkg" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef STRAIN_PID_H_
#define STRAIN_PID_H_

#include <gaze_chain/pid.h>

template<class __StorageDataType = double, class __TimeDataType = double>
class StrainPid : public Pid<__StorageDataType, __TimeDataType, 2>
{
public:
	typedef __StorageDataType _StorageDataType;
	typedef __TimeDataType _TimeDataType;

	typedef Pid<_StorageDataType, _TimeDataType, 2> _Pid;
	typedef typename _Pid::_Settings _Settings;

	struct ErrorMode
	{
		typedef unsigned int _DataType;

		// minimize incoming angle error
		const static _DataType ANGLE = 0;
		// minimize accumulated strain
		const static _DataType STRAIN = 1;
	};

protected:
	_StorageDataType strain_min_ceiling, strain_min_floor, strain_max_floor, strain_max_ceiling;
	_StorageDataType total_strain;
	typename ErrorMode::_DataType error_mode;

public:
	StrainPid( _Settings pid_settings, _Settings strain_settings, _StorageDataType strain_min_ceiling_ = -100, _StorageDataType strain_min_floor_ = -1, _StorageDataType strain_max_floor_ = 1, _StorageDataType strain_max_ceiling_ = 100, _StorageDataType output_ = 0 ) :
	_Pid (), strain_min_ceiling( strain_min_ceiling_ ), strain_min_floor( strain_min_floor_ ), strain_max_floor( strain_max_floor_ ), strain_max_ceiling( strain_max_ceiling_ ), total_strain( 0 ), error_mode( ErrorMode::ANGLE )
	{
		applySettings( ErrorMode::ANGLE, pid_settings );
		applySettings( ErrorMode::STRAIN, strain_settings );
	}

	_StorageDataType update( _StorageDataType error, _StorageDataType strain, _TimeDataType dt = -1 )
	{
		static _StorageDataType error_value = 0;
		static _StorageDataType max_error_value = 0;
		static _TimeDataType last_time = getTimeInSecs();
		static _TimeDataType current_time = getTimeInSecs();

		last_time = current_time;
		current_time = getTimeInSecs();

		printf( "mode: %u, total_strain: %f\n", error_mode, total_strain );

		dt = dt > 0 ? dt : ( current_time - last_time );

		// here, i_min and i_max define the maximum strain the joint will allow before trying to minimize strain
		if( error_mode == ErrorMode::ANGLE && ( total_strain <= strain_min_ceiling || total_strain >= strain_max_ceiling ) )
		{
			error_mode = ErrorMode::STRAIN;
			max_error_value = strain;
			this->pids[error_mode].reset();

		}
		else if( error_mode == ErrorMode::STRAIN && total_strain <= strain_max_floor && total_strain >= strain_min_floor )
		{
			error_mode = ErrorMode::ANGLE;
			max_error_value = error;
			this->pids[error_mode].reset();
		}

		switch( error_mode )
		{
			case ErrorMode::ANGLE:
			error_value = error;
			total_strain += strain * dt;
			break;
			case ErrorMode::STRAIN:
			error_value = strain;
			total_strain -= (max_error_value - error_value) * dt * 5;
			break;
		}

		return this->pids[error_mode].update( error_value, dt );
	}

	virtual ~StrainPid()
	{
		//
	}

};

#endif /* STRAIN_PID_H_ */
