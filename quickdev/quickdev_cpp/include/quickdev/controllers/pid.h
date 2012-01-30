/***************************************************************************
 *  include/quickdev/controllers/pid.h
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

#ifndef QUICKDEVCPP_QUICKDEV_CONTROLLERS_PID_H_
#define QUICKDEVCPP_QUICKDEV_CONTROLLERS_PID_H_

#include <quickdev/time.h>
#include <quickdev/auto_bind.h>
#include <stdio.h>
#include <array>
#include <boost/shared_ptr.hpp>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

class PIDSettings
{
public:
    typedef double _Data;

    _Data p_, i_, d_, output_min_, output_max_;

    typedef std::function<void()> _SettingsChangedCallback;
    _SettingsChangedCallback settings_changed_callback_;

    PIDSettings( _Data const & p = 0, _Data const & i = 0, _Data const & d = 0, _Data const & output_min = 0, _Data const & output_max = 0 )
    :
        p_( p ), i_( i ), d_( d ), output_min_( output_min ), output_max_( output_max )
    {
        //
    }

    void registerSettingsChangedCallback( _SettingsChangedCallback const & callback )
    {
        settings_changed_callback_ = callback;
    }
};

template<class __Settings = PIDSettings>
class PIDBase
{
public:
    typedef __Settings _Settings;
    typedef boost::shared_ptr<__Settings> _SettingsPtr;
    typedef double _Data;

protected:
    _SettingsPtr settings_;

public:
    PIDBase(){}

    PIDBase( _SettingsPtr settings )
    :
        settings_( settings )
    {
        settings_->registerSettingsChangedCallback( quickdev::auto_bind( &PIDBase::settingsChangedCB, this ) );
    }

private:
    double desired_value_;
    double observed_value_;
    double output_value_;

    double i_term_;

    double last_observed_value_;

    bool is_initialized_;

    Timer timer_;

public:
    double const & update()
    {
        if( !is_initialized_ )
        {
            is_initialized_ = true;
            return output_value_;
        }

        auto const dt = timer_.update();
        double const error = desired_value_ - observed_value_;

        auto const & p = settings_->p_;
        auto const & i = settings_->i_ * dt;
        auto const & d = settings_->d_ / dt;
        auto const & outputMin = settings_->output_min_;
        auto const & outputMax = settings_->output_max_;

        i_term_ += i * error;
        if( i_term_ > outputMax ) i_term_ = outputMax;
        else if( i_term_ < outputMin ) i_term_ = outputMin;

        auto const dInput = observed_value_ - last_observed_value_;

        /*Compute PID Output*/
        output_value_ = p * error + i_term_ - d * dInput;
        if( output_value_ > outputMax ) output_value_ = outputMax;
        else if( output_value_ < outputMin ) output_value_ = outputMin;

        /*Remember some variables for next time*/
        last_observed_value_ = observed_value_;

        return output_value_;
    }

    void setDesiredValue( _Data const & desired_value )
    {
        desired_value_ = desired_value;
    }

    void setObservedValue( _Data const & observed_value )
    {
        observed_value_ = observed_value;
    }

    void applySettings( _SettingsPtr settings )
    {
        if( settings->output_min_ > settings->output_max_ || settings->output_max_ < settings->output_min_ ) return;

        if( &settings != &settings_ )
        {
            settings_ = settings;
            settings_->registerSettingsChangedCallback( quickdev::auto_bind( &PIDBase::settingsChangedCB, this ) );
        }

        if( output_value_ < settings_->output_min_ ) output_value_ = settings_->output_min_;
        if( i_term_ < settings_->output_min_ ) i_term_ = settings_->output_min_;

        if( output_value_ > settings_->output_max_ ) output_value_ = settings_->output_max_;
        if( i_term_ > settings_->output_max_ ) i_term_ = settings_->output_max_;
    }

    void applySettings()
    {
        applySettings( settings_ );
    }

    void settingsChangedCB()
    {
        applySettings();
    }
};

template<unsigned int __Dim__, class __Settings>
class PID
{
public:
    typedef __Settings _Settings;
    typedef boost::shared_ptr<__Settings> _SettingsPtr;
    typedef std::array<_SettingsPtr, __Dim__> _SettingsArray;

    std::array<PIDBase<__Settings>, __Dim__> pids_;

    PID(){}

    template<class... __Args>
    PID( __Args... args )
    {
        applySettings( args... );
    }

    void applySettings( _SettingsArray const & settings )
    {
        for( size_t i = 0; i < settings.size(); ++i )
        {
            pids_[i].applySettings( settings[i] );
        }
    }

    template<
        class... __Args,
        typename std::enable_if<(sizeof...(__Args) == __Dim__), int>::type = 0
    >
    void applySettings( __Args... args )
    {
        _SettingsArray const settings = { { args... } };
        applySettings( settings );
    }
};

template<class __Settings>
class PID<1, __Settings> : public PIDBase<__Settings>
{
public:

};

template<class __Settings>
class PID<3, __Settings>
{
public:
    typedef typename PIDBase<__Settings>::_Data _Data;
    typedef __Settings _Settings;
    typedef boost::shared_ptr<__Settings> _SettingsPtr;
    typedef PIDBase<__Settings> _PID;
    typedef std::array<_SettingsPtr, 3> _SettingsArray;

    std::array<_PID, 3> pids_;

    _PID & x_, y_, z_;

    PID()
    :
        x_( pids_[0] ), y_( pids_[1] ), z_( pids_[2] )
    {
        //
    }

    template<class... __Args>
    PID( __Args... args )
    :
        x_( pids_[0] ), y_( pids_[1] ), z_( pids_[2] )
    {
        applySettings( args... );
    }

    void applySettings( _SettingsArray const & settings )
    {
        for( size_t i = 0; i < settings.size(); ++i )
        {
            pids_[i].applySettings( settings[i] );
        }
    }

    template<
        class... __Args,
        typename std::enable_if<(sizeof...(__Args) == 3), int>::type = 0
    >
    void applySettings( __Args... args )
    {
        // apparently it's "better form" to specify the initializer list with {{...}}
        _SettingsArray const settings = { { args... } };
        applySettings( settings );
    }
};

template<class __Settings>
class PID<6, __Settings>
{
public:
    typedef __Settings _Settings;
    typedef boost::shared_ptr<__Settings> _SettingsPtr;
    typedef std::array<_SettingsPtr, 6> _SettingsArray;

    PID<3, __Settings> linear_;
    PID<3, __Settings> angular_;

    PID(){}

    template<class... __Args>
    PID( __Args... args )
    {
        applySettings( args... );
    }

    void applySettings( _SettingsArray const & settings )
    {
        for( size_t i = 0; i < 3; ++i )
        {
            linear_.pids_[i].applySettings( settings[i] );
        }

        for( size_t i = 0; i < 3; ++i )
        {
            angular_.pids_[i].applySettings( settings[i+3] );
        }
    }

    template<
        class... __Args,
        typename std::enable_if<(sizeof...(__Args) == 6), int>::type = 0
    >
    void applySettings( __Args... args )
    {
        _SettingsArray const settings = { { args... } };
        applySettings( settings );
    }
};

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_CONTROLLERS_PID_H_
