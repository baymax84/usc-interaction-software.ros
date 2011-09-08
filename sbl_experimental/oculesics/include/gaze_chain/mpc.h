/*******************************************************************************
 *
 *      mpc
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

#ifndef MPC_H_
#define MPC_H_

#include <vector>
#include <interaction_common/time.h>
#include <stdio.h>

namespace MultiplexedControllerParams
{
	namespace OutputMode
	{
		const unsigned int SIGNAL = 0;
		const unsigned int INDEX = 1;
	}
}

template<class __SignalValueDataType = double, unsigned int __Dim__ = 1, unsigned int __OutputMode__ = MultiplexedControllerParams::OutputMode::SIGNAL>
class MultiplexedControllerBase
{
public:
	typedef __SignalValueDataType _SignalValueDataType;

	typedef double _TimeDataType;

	struct ReliefSignal
	{
		_SignalValueDataType strain;
		_SignalValueDataType relief;

		ReliefSignal() : strain( 0 ), relief( 0 )
		{
			//
		}

		ReliefSignal( _SignalValueDataType strain_, _SignalValueDataType releif_ ) :
			strain( strain_ ), relief( releif_ )
		{
			//
		}

		ReliefSignal( _SignalValueDataType strain_ ) :
			strain( strain_ ), relief( strain_ )
		{
			//
		}

	};

	typedef ReliefSignal _InputDataType;
	typedef std::vector<_InputDataType> _InputVector;

	struct Settings
	{
		// min_ceiling < min_floor < max_floor < max_ceiling
		_SignalValueDataType min_ceiling, min_floor, max_floor, max_ceiling;

		Settings( _SignalValueDataType min_ceiling_ = 0, _SignalValueDataType min_floor_ = 0, _SignalValueDataType max_floor_ = 0, _SignalValueDataType max_ceiling_ = 0 ) :
			min_ceiling( min_ceiling_ ), min_floor( min_floor_ ), max_floor( max_floor_ ), max_ceiling( max_ceiling_ )
		{
			//
		}
	};

	typedef Settings _Settings;

	struct ErrorComponent
	{
		_Settings settings;
		_SignalValueDataType last_error;
		_SignalValueDataType rate_of_change;
		_SignalValueDataType raw_error;
		_SignalValueDataType integrated_error;
		_SignalValueDataType activation_value;

		// signal has been activated
		bool active;

		// this is the currently selected signal
		bool selected;

		_SignalValueDataType last_integrated_error;

		ErrorComponent( _Settings settings_ = _Settings(), _SignalValueDataType integrated_error_ = 0 ) :
			settings( settings_ ), selected( false ), last_integrated_error( integrated_error_ )
		{
			// set and store the initial error (to be restored on subsequent calls to reset() )
			reset( integrated_error_ );
		}

		void reset( _SignalValueDataType init_integrated_error_ = 0 )
		{
			static _SignalValueDataType init_integrated_error = init_integrated_error_;
			integrated_error = init_integrated_error;
			//last_error = 0;
			activation_value = 0;
			active = false;
		}

		bool outOfUpperBounds()
		{
			return integrated_error <= settings.min_ceiling || integrated_error >= settings.max_ceiling;
		}

		bool outOfLowerBounds()
		{
			return integrated_error >= settings.min_floor && integrated_error <= settings.max_floor;
		}

		bool outOfBounds()
		{
			return outOfUpperBounds() || outOfLowerBounds();
		}

		bool aboveCeiling()
		{
			return outOfUpperBounds();
		}

		bool belowFloor()
		{
			//           min_ceiling  min_floor     max_floor  max_ceiling
			// <--------------|-----------|------0------|-----------|-------------->
			// [above ceiling] [in bounds] [below floor] [in bounds] [above ceiling]
			//
			// if we're trying to decrease the error magnitude and we pass through the floor and into the bounds
			// of the error of the opposite direction in a single step, we will fail to recognize that the error
			// actually passed through the floor unless we know the rate of change of the error, specifically
			// the direction of the rate of change of the error
			return outOfLowerBounds();// || ( rate_of_change > 0 && integrated_error >= settings.min_floor ) || ( rate_of_change < 0 && integrated_error <= settings.max_floor );
		}

		bool isTriggered()
		{
			return active || aboveCeiling();
		}

		// calling update() will just return integrated_error
		_SignalValueDataType & update( _SignalValueDataType error = 0, _TimeDataType dt = 0 )
		{
			if ( active && belowFloor() )
			{
				reset();
			}

			raw_error = error;

			if ( dt > 0 )
			{
				printf( "raw error value: %f\n", error );
				error = active ? error - activation_value : error;
				//error = active ? -error : error;
				rate_of_change = ( error - last_error );
				integrated_error += error * dt;

				// make sure the integrated error doesn't go outside the specified bounds
				if ( integrated_error < settings.min_ceiling ) integrated_error = settings.min_ceiling;
				if ( integrated_error > settings.max_ceiling ) integrated_error = settings.max_ceiling;
			}

			printf( "error component update( [%u %u %u] %f %f [%f %f] %f )\n", isTriggered(), active, selected, error, activation_value, integrated_error, last_integrated_error, dt );

			if ( !active && isTriggered() )
			{
				active = true;
				activation_value = error;
			}

			last_error = error;
			last_integrated_error = integrated_error;

			return integrated_error;
		}

		_SignalValueDataType getSolution()
		{
			return -raw_error;
		}
	};

	typedef ErrorComponent _ErrorComponent;

	struct Signal
	{
		_ErrorComponent error_component;
		bool enabled;

		Signal( _ErrorComponent error_component_ = _ErrorComponent(), bool enabled_ = true, bool selected_ = false ) :
			error_component( error_component_ ), enabled( enabled_ )
		{
			selected() = selected_;
		}

		_SignalValueDataType update( _InputDataType error, _TimeDataType dt )
		{
			return error_component.update( error_component.active ? error.relief : error.strain, dt );
		}

		_SignalValueDataType getSolution()
		{
			return error_component.getSolution();
		}

		bool & selected()
		{
			return error_component.selected;
		}

		const bool & selected() const
		{
			return error_component.selected;
		}
	};

	typedef Signal _Signal;

protected:
	_Signal signals[__Dim__];

public:
	MultiplexedControllerBase( std::vector<_Settings> settings_ )
	{
		for ( unsigned int i = 0; i < settings_.size(); ++i )
		{
			signals[i].error_component.settings = settings_[i];
		}
	}

	virtual ~MultiplexedControllerBase()
	{
		//
	}

	const _Signal & operator[]( unsigned int index ) const
	{
		return signals[index];
	}

	_Signal & operator[]( unsigned int index )
	{
		return signals[index];
	}

	unsigned int selectSignalIndex()
	{
		int selected_index = -1;
		for ( unsigned int i = 0; i < __Dim__; ++i )
		{
			_Signal & current_signal = signals[i];
			if ( selected_index < 0 && current_signal.enabled && current_signal.error_component.isTriggered() )
			{
				printf( "selecting signal %u\n", i );
				selected_index = i;
			}
			else current_signal.selected() = false;
		}

		if ( selected_index < 0 )
		{
			printf( "selecting default signal\n" );
			selected_index = 0;
		}

		signals[selected_index].selected() = true;
		return selected_index;
	}

	_Signal & selectSignal()
	{
		return signals[selectSignalIndex()];
	}

	void setSignalStates( std::vector<bool> signal_states )
	{
		for ( unsigned int i = 0; i < __Dim__; ++i )
		{
			setSignalState( i, signal_states[i] );
		}
	}

	void setSignalState( unsigned int i, bool enabled )
	{
		signals[i].enabled = enabled;
	}
};

template<class __SignalValueDataType = double, unsigned int __Dim__ = 1, unsigned int __OutputMode__ = MultiplexedControllerParams::OutputMode::SIGNAL>
class MultiplexedController : public MultiplexedControllerBase<__SignalValueDataType, __Dim__, __OutputMode__> {
public:
typedef MultiplexedControllerBase<__SignalValueDataType, __Dim__, __OutputMode__> _MultiplexedControllerBase;

typedef typename _MultiplexedControllerBase::_Settings _Settings;
typedef typename _MultiplexedControllerBase::_TimeDataType _TimeDataType;
typedef typename _MultiplexedControllerBase::_InputDataType _InputDataType;
typedef typename _MultiplexedControllerBase::_InputVector _InputVector;
typedef typename _MultiplexedControllerBase::_Signal _Signal;

MultiplexedController( std::vector<_Settings> settings_ ) : _MultiplexedControllerBase( settings_ )
{

}
};

template<class __SignalValueDataType, unsigned int __Dim__>
class MultiplexedController<__SignalValueDataType, __Dim__ ,MultiplexedControllerParams::OutputMode::INDEX> : public MultiplexedControllerBase<__SignalValueDataType,  __Dim__, MultiplexedControllerParams::OutputMode::INDEX>
{
public:
	typedef MultiplexedControllerBase<__SignalValueDataType, __Dim__, MultiplexedControllerParams::OutputMode::INDEX> _MultiplexedControllerBase;

	typedef typename _MultiplexedControllerBase::_Settings _Settings;
	typedef typename _MultiplexedControllerBase::_TimeDataType _TimeDataType;
	typedef typename _MultiplexedControllerBase::_InputDataType _InputDataType;
	typedef typename _MultiplexedControllerBase::_InputVector _InputVector;
	typedef typename _MultiplexedControllerBase::_Signal _Signal;

	MultiplexedController( std::vector<_Settings> settings_ ) : _MultiplexedControllerBase( settings_ )
	{

	}

	unsigned int update( _InputVector error, _TimeDataType dt = 0 )
	{
		static _TimeDataType last_time = 0;
		static _TimeDataType current_time = getTimeInSecs();

		last_time = current_time;
		current_time = getTimeInSecs();

		dt = dt > 0 ? dt : ( current_time - last_time );

		unsigned int signal_index = this->selectSignalIndex();

		for ( unsigned int i = 0; i < __Dim__; ++i )
		{
			this->signals[i].update( error[i], dt );
		}

		return signal_index;
	}
};

template<class __SignalValueDataType, unsigned int __Dim__>
class MultiplexedController<__SignalValueDataType, __Dim__ ,MultiplexedControllerParams::OutputMode::SIGNAL> : public MultiplexedControllerBase<__SignalValueDataType,  __Dim__, MultiplexedControllerParams::OutputMode::SIGNAL>
{
public:
	typedef MultiplexedControllerBase<__SignalValueDataType, __Dim__, MultiplexedControllerParams::OutputMode::SIGNAL> _MultiplexedControllerBase;

	typedef typename _MultiplexedControllerBase::_Settings _Settings;
	typedef typename _MultiplexedControllerBase::_TimeDataType _TimeDataType;
	typedef typename _MultiplexedControllerBase::_InputDataType _InputDataType;
	typedef typename _MultiplexedControllerBase::_InputVector _InputVector;
	typedef typename _MultiplexedControllerBase::_Signal _Signal;

	MultiplexedController( std::vector<_Settings> settings_ ) : _MultiplexedControllerBase( settings_ )
	{

	}

	__SignalValueDataType update( _InputVector error, _TimeDataType dt = 0 )
	{
		static _TimeDataType last_time = 0;
		static _TimeDataType current_time = getTimeInSecs();

		last_time = current_time;
		current_time = getTimeInSecs();

		dt = dt > 0 ? dt : ( current_time - last_time );

		_Signal & signal = this->selectSignal();

		for ( unsigned int i = 0; i < __Dim__; ++i )
		{
			this->signals[i].update( error[i], dt );
		}

		return signal.getSolution();
	}
};

#endif /* MPC_H_ */
