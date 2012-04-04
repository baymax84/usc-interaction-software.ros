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
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>
#include <map>
#include <sstream>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

QUICKDEV_DECLARE_POLICY( Joystick, NodeHandlePolicy )

QUICKDEV_DECLARE_POLICY_CLASS( Joystick )
{
    QUICKDEV_MAKE_POLICY_FUNCS( Joystick )

public:
    typedef joy::Joy _JoystickMsg;
    typedef geometry_msgs::Twist _VelocityMsg;

    struct Axis
    {
        typedef std::string _Name;
        typedef int _Index;
        typedef double _Scale;
        typedef double _Value;

        _Name name_;
        _Index index_;
        _Scale scale_;
        bool is_button_;

        Axis( _Name name = _Name(), _Index index = _Index(), _Scale scale = _Scale(), bool is_button = false )
        :
            name_( name ), index_( index ), scale_( scale ), is_button_( is_button )
        {
            //
        }

        inline _Value getRawValue( _JoystickMsg::ConstPtr const & msg ) const
        {
            return msg ? ( is_button_ ? msg->buttons[index_] : msg->axes[index_] ) : _Value();
        }

        inline _Value getValue( _JoystickMsg::ConstPtr const & msg ) const
        {
            return scale_ * getRawValue( msg );
        }

        inline _Value getValueAsButton( _JoystickMsg::ConstPtr const & msg ) const
        {
            return is_button_ ? getRawValue( msg ) : getValue( msg ) > 0.75;
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
        joystick_timeout_ = ros::ParamReader<double, 1>::readParam( nh_rel, "keep_alive_period", 1.0 );

        // read as many axis#_index values as exist on the parameter server starting with axis0_index
        auto const axis_indices( ros::ParamReader<_Axis::_Index, 0>::readParams( nh_rel, params_namespace + "axis", "_index", 0 ) );
        auto const axis_names( ros::ParamReader<_Axis::_Name, 0>::readParams( nh_rel, params_namespace + "axis", "_name", 0 ) );
        auto const axis_scales( ros::ParamReader<_Axis::_Scale, 0>::readParams( nh_rel, params_namespace + "axis", "_scale", 0 ) );

        auto const button_indices( ros::ParamReader<_Axis::_Index, 0>::readParams( nh_rel, params_namespace + "button", "_index", 0 ) );
        auto const button_names( ros::ParamReader<_Axis::_Name, 0>::readParams( nh_rel, params_namespace + "button", "_name", 0 ) );

        // unroll the axis names and indices into a map from names to indices
        for( size_t i = 0; i < axis_names.size() && i < axis_indices.size(); ++i )
        {
            _Axis::_Name const & axis_name( axis_names[i] );
            _Axis::_Index const & axis_index( axis_indices[i] );
            _Axis::_Scale const & axis_scale( axis_scales.size() > i ? axis_scales[i] : _Axis::_Scale( 1.0 ) );
            _Axis const axis( axis_name, axis_index, axis_scale );

            PRINT_INFO( "Adding axis: %s", axis.str().c_str() );
            axes_map_[axis_name] = axis;
        }

        // unroll the button names and indices into a map from names to indices
        for( size_t i = 0; i < button_names.size() && i < button_indices.size(); ++i )
        {
            _Axis::_Name const & axis_name( button_names[i] );
            _Axis::_Index const & axis_index( button_indices[i] );
            _Axis const axis( axis_name, axis_index, 1.0, true );

            PRINT_INFO( "Adding axis: %s", axis.str().c_str() );
            axes_map_[axis_name] = axis;
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
