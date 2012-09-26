/***************************************************************************
 *  include/virtual_camera/simple_virtual_camera_node.h
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
 *  * Neither the name of simulated-sensing nor the names of its
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

#ifndef VIRTUALCAMERA_SIMPLEVIRTUALCAMERANODE_H_
#define VIRTUALCAMERA_SIMPLEVIRTUALCAMERANODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/image_proc_policy.h>

// objects
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>
#include <camera_info_manager/camera_info_manager.h>

// utils
#include <quickdev/geometry_message_conversions.h>

// msgs
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/CameraInfo.h>

typedef quickdev::ImageProcPolicy _ImageProcPolicy;

typedef visualization_msgs::Marker _MarkerMsg;
typedef visualization_msgs::MarkerArray _MarkerArrayMsg;
typedef sensor_msgs::CameraInfo _CameraInfoMsg;

typedef camera_info_manager::CameraInfoManager _CameraInfoManager;

// Declare a node called SimpleVirtualCameraNode.
// A quickdev::RunablePolicy is automatically prepended to the list of policies our node will use.
// To use more policies, simply list them here:
//
// QUICKDEV_DECLARE_NODE( SimpleVirtualCamera, SomePolicy1, SomePolicy2 )
//
QUICKDEV_DECLARE_NODE( SimpleVirtualCamera, _ImageProcPolicy )

// Declare a class called SimpleVirtualCameraNode
//
QUICKDEV_DECLARE_NODE_CLASS( SimpleVirtualCamera )
{
protected:
    ros::MultiSubscriber<> multi_sub_;
    ros::MultiPublisher<> multi_pub_;

    _CameraInfoManager camera_info_manager_;
    _CameraInfoMsg::ConstPtr camera_info_msg_ptr_;

    // Variable initializations can be appended to this constructor as a comma-separated list:
    //
    // QUICKDEV_DECLARE_NODE_CONSTRUCTOR( SimpleVirtualCamera ), member1_( some_value ), member2_( some_other_value ){}
    //
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( SimpleVirtualCamera ),
        camera_info_manager_( quickdev::getFirstOfType<ros::NodeHandle>( args... ) )
    {
        //
    }

    // This function is called by quickdev::RunablePolicy after all policies are constructed but just before the main loop is started.
    // All policy initialization should be done here.
    //
    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_pub_.addPublishers<_CameraInfoMsg>( nh_rel, { "camera_info" } );

        std::string node_name = nh_rel.getNamespace();

        // strip leading '/' from node name, if applicable
        if( node_name.substr( 0, 1 ) == "/" ) node_name = node_name.substr( 1 );
        camera_info_manager_.setCameraName( node_name );

        auto const camera_info_url = quickdev::ParamReader::readParam<std::string>( nh_rel, "camera_info_url" );

        if( camera_info_manager_.validateURL( camera_info_url ) ) camera_info_manager_.loadCameraInfo( camera_info_url );
        else PRINT_WARN( "Camera info URL %s not valid.", camera_info_url.c_str() );

        _CameraInfoMsg camera_info_msg = camera_info_manager_.getCameraInfo();
        camera_info_msg.header.frame_id = node_name;

        camera_info_msg_ptr_ = quickdev::make_const_shared( camera_info_msg );

        initPolicies<_ImageProcPolicy>
        (
            "output_image_topic_param", std::string( "image_raw" )
        );

        initPolicies<quickdev::policy::ALL>();
    }

    // This optional function is called by quickdev::RunablePolicy at a fixed rate (defined by the ROS param _loop_rate).
    // Most updateable policies should have their update( ... ) functions called within this context.
    //
    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // VIRTUALCAMERA_SIMPLEVIRTUALCAMERANODE_H_
