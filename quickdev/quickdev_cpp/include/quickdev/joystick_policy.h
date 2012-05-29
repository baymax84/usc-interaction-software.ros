/***************************************************************************
 *  include/quickdev/joystick_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_JOYSTICKPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_JOYSTICKPOLICY_H_

#include <quickdev/node_handle_policy.h>
#include <quickdev/storage_adapter.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>
#include <geometry_msgs/Twist.h>
#include <map>
#include <sstream>

using quickdev::types::_JoystickMsg;

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{
class Axis
{
public:
    typedef std::string _Name;
    typedef int _Index;
    typedef double _Scale;
    typedef double _Value;

    _Name name_;
    _Index index_;
    _Scale scale_;
    bool is_button_;
    bool is_button_pair_;

    Axis* button1_;
    Axis* button2_;

    Axis( _Name const & name = _Name(), _Index const & index = -1, _Scale const & scale = 1.0, bool const & is_button = false )
    :
        name_( name ), index_( index ), scale_( scale ), is_button_( is_button ), is_button_pair_( false ), button1_( NULL ), button2_( NULL )
    {
        //
    }

    Axis( _Name const & name, Axis & button1, Axis & button2, _Scale const & scale = 1.0 )
    :
        name_( name ), index_( -1 ), scale_( scale ), is_button_( false ), is_button_pair_( true ), button1_( &button1 ), button2_( &button2 )
    {
        //
    }

    _Value getRawValue( _JoystickMsg::ConstPtr const & msg ) const
    {
        if( msg )
        {
            if( is_button_ )
            {
                if( index_ >= 0 ) return msg->buttons[index_];
                return _Value();
            }
            else if( is_button_pair_ ) return getValueAsButtonPair( msg );
            return msg->axes[index_];
        }
        return _Value();
    }

    _Value getValue( _JoystickMsg::ConstPtr const & msg ) const
    {
        return scale_ * getRawValue( msg );
    }

    _Value getValueAsButton( _JoystickMsg::ConstPtr const & msg ) const
    {
        return is_button_ ? getRawValue( msg ) : getValue( msg ) > 0.75;
    }

    _Value getValueAsButtonPair( _JoystickMsg::ConstPtr const & msg ) const
    {
        if( is_button_pair_ )
        {
            return button1_->getValue( msg ) - button2_->getValue( msg );
        }
        return _Value();
    }

    std::string str() const
    {
        std::stringstream ss;
        ss << "[ " << index_ << " : " << name_;

        if( is_button_ ) ss << " (button)";
        else ss << " * " << scale_;

        ss << " ]";
        return ss.str();
    }
};

QUICKDEV_DECLARE_POLICY( Joystick, NodeHandlePolicy )

QUICKDEV_DECLARE_POLICY_CLASS( Joystick )
{
    QUICKDEV_MAKE_POLICY_FUNCS( Joystick )

public:
    typedef geometry_msgs::Twist _VelocityMsg;

    typedef Axis _Axis;
    typedef std::map<_Axis::_Name, _Axis> _AxesMap;

private:
    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;

    _VelocityMsg::Ptr velocity_msg_;

    _JoystickMsg::ConstPtr last_joystick_message_;
    ros::Time last_joystick_message_time_;
    bool enabled_;
    double joystick_timeout_;
    std::string cmd_vel_topic_name_;

    _AxesMap axes_map_;


    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( Joystick ),
        last_joystick_message_time_( ros::Time::now() ),
        enabled_( false ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );

        //preInit();

        printPolicyActionDone( "create", this );
    }

private:
    void postInit()
    {
        auto & nh_rel = NodeHandlePolicy::getNodeHandle();

        multi_pub_.addPublishers<geometry_msgs::Twist>( nh_rel, { cmd_vel_topic_name_ } );
        multi_sub_.addSubscriber( nh_rel, "joystick", &JoystickPolicy::joystickCB_0, this );
    }

public:
    QUICKDEV_ENABLE_INIT()
    {

        auto & nh_rel = NodeHandlePolicy::getNodeHandle();

        auto robot_name = policy::readPolicyParam<std::string>( nh_rel, "robot_name_param", "robot_name", "", std::forward<__Args>( args )... );
        if( robot_name.size() > 0 ) robot_name.insert( 0, "/" );

        cmd_vel_topic_name_ = getMetaParamDef<std::string>( "cmd_vel_topic_name_param", robot_name.size() > 0 ? robot_name + "/cmd_vel" : "cmd_vel", std::forward<__Args>( args )... );

        auto params_namespace = policy::readPolicyParam<std::string>( nh_rel, "namespace_name_param", "namespace_name", "joystick", std::forward<__Args>( args )... );

        if( !params_namespace.empty() && params_namespace.substr( params_namespace.size() - 1, 1 ) != "/" ) params_namespace += "/";

        // if we don't get any joystick messages after this much time, zero out all fields of the outgoing velocity message
        joystick_timeout_ = quickdev::ParamReader::readParam<double>( nh_rel, "keep_alive_period", 1.0 );

        // read axis and button mappings
        auto axes_param = quickdev::ParamReader::readParam<XmlRpc::XmlRpcValue>( nh_rel, params_namespace + "axes" );
        auto buttons_param = quickdev::ParamReader::readParam<XmlRpc::XmlRpcValue>( nh_rel, params_namespace + "buttons" );

        // store button mappings
        for( auto buttons_it = buttons_param.begin(); buttons_it != buttons_param.end(); ++buttons_it )
        {
            std::string const & axis_name = buttons_it->first;
            int const & axis_index = buttons_it->second;

            axes_map_[axis_name] = _Axis( axis_name, axis_index, 1.0, true );
        }

        // store axis mappings
        for( auto axes_it = axes_param.begin(); axes_it != axes_param.end(); ++axes_it )
        {
            std::string const & axis_name = axes_it->first;
            auto & axis = axes_it->second;
            double const & axis_scale = axis["scale"];

            if( !axis.hasMember( "index" ) )
            {
                auto button_negative = axes_map_.find( axis["negative"] );
                auto button_positive = axes_map_.find( axis["positive"] );
                if( button_positive != axes_map_.end() && button_negative != axes_map_.end() ) axes_map_[axis_name] = _Axis( axis_name, button_positive->second, button_negative->second, axis_scale );
            }
            else
            {
                int const & axis_index = axes_it->second["index"];
                axes_map_[axis_name] = _Axis( axis_name, axis_index, axis_scale );
            }
        }

        postInit();

        QUICKDEV_SET_INITIALIZED();
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( joystickCB_0, _JoystickMsg )
    {
        last_joystick_message_ = msg;
        last_joystick_message_time_ = ros::Time::now();

        joystickCB( msg );
    }

    virtual QUICKDEV_DECLARE_MESSAGE_CALLBACK( joystickCB, _JoystickMsg ){}

    bool updateVelocityComponent( _Axis::_Name const  & axis_name, _Axis::_Value & axis_value )
    {
        if( !last_joystick_message_ ) return false;

        auto const & axis_it = axes_map_.find( axis_name );
        if( axis_it == axes_map_.end() ) return false;

        _Axis const & axis = axis_it->second;

        if( !enabled_ || ( ros::Time::now() - last_joystick_message_time_ ).toSec() > joystick_timeout_ ) axis_value = 0;
        else axis_value = axis.getValue( last_joystick_message_ );

        return true;
    }

    auto getVelocityMsg() -> decltype( velocity_msg_ ) &
    {
        return velocity_msg_;
    }

    bool axisExists( _Axis::_Name const & axis_name ) const
    {
        return axes_map_.count( axis_name );
    }

    _Axis const & getAxis( _Axis::_Name const & axis_name ) const
    {
        static _Axis const default_axis = _Axis();

        auto const & axis_it = axes_map_.find( axis_name );
        if( axis_it == axes_map_.end() ) return default_axis;
        return axis_it->second;
    }

    _Axis::_Value getAxisValue( _Axis::_Name const & axis_name ) const
    {
        auto const & axis = getAxis( axis_name );
        return axis.getValue( last_joystick_message_ );
    }

    void update( bool const & auto_publish = true )
    {
        QUICKDEV_ASSERT_INITIALIZED();

        velocity_msg_ = geometry_msgs::Twist::Ptr( new geometry_msgs::Twist );

        auto const & enable_axis_it = axes_map_.find( "enable" );
        enabled_ = enable_axis_it == axes_map_.end() || enable_axis_it->second.getValue( last_joystick_message_ ) > 0;

        updateVelocityComponent( "linear.x", velocity_msg_->linear.x );
        updateVelocityComponent( "linear.y", velocity_msg_->linear.y );
        updateVelocityComponent( "linear.z", velocity_msg_->linear.z );

        updateVelocityComponent( "angular.x", velocity_msg_->angular.x );
        updateVelocityComponent( "angular.y", velocity_msg_->angular.y );
        updateVelocityComponent( "angular.z", velocity_msg_->angular.z );

        if( auto_publish ) publish();
    }

    void publish() const
    {
        multi_pub_.publish( cmd_vel_topic_name_, velocity_msg_ );
    }

    inline bool const & isEnabled() const
    {
        return enabled_;
    }
};

}

#endif // QUICKDEVCPP_QUICKDEV_JOYSTICKPOLICY_H_
