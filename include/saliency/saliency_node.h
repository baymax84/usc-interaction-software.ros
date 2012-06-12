/***************************************************************************
 *  include/saliency/saliency_node.h
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
 *  * Neither the name of interaction-ros-pkg nor the names of its
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

#ifndef SALIENCY_SALIENCYNODE_H_
#define SALIENCY_SALIENCYNODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/image_proc_policy.h>

// envision
#include <envision/env_alloc.h>
#include <envision/env_c_math_ops.h>
#include <envision/env_image.h>
#include <envision/env_image_ops.h>
#include <envision/env_log.h>
#include <envision/env_mt_visual_cortex.h>
#include <envision/env_params.h>
#include <envision/env_pthread_interface.h>
#include <envision/env_stdio_interface.h>
#include <envision/env_visual_cortex.h>

#include <envision/env_opencv.h>

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h> // for atoi(), malloc(), free()

// ######################################################################
// Thunk to convert from env_size_t to size_t
static void* malloc_thunk( env_size_t n )
{
        return malloc(n);
}

typedef quickdev::ImageProcPolicy _ImageProcPolicy;

QUICKDEV_DECLARE_NODE( Saliency, _ImageProcPolicy )

QUICKDEV_DECLARE_NODE_CLASS( Saliency )
{
    size_t num_threads_;

    env_params env_params_;
    env_visual_cortex visual_cortex_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( Saliency )
    {
        initPolicies<QUICKDEV_GET_RUNABLE_POLICY()>();
    }

    ~SaliencyNode()
    {
        env_visual_cortex_destroy( &visual_cortex_ );
        env_allocation_cleanup();
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        num_threads_ = quickdev::ParamReader::readParam<int>( nh_rel, "num_image_threads", 4 );

        env_params_set_defaults( &env_params_ );

        env_params_.maxnorm_type = ENV_VCXNORM_MAXNORM;
        env_params_.scale_bits = 16;

        env_assert_set_handler( &env_stdio_assert_handler );

        if( num_threads_ )
        {
            env_init_pthread_alloc();
            env_init_pthread_job_server();
        }
        env_allocation_init( &malloc_thunk, &free );

        env_visual_cortex_init( &visual_cortex_, &env_params_ );

        initPolicies<_ImageProcPolicy>
        (
            "image_callback_param", quickdev::auto_bind( &SaliencyNode::imageCB, this ),
            "output_image_topic_param", std::string( "saliency_image" )
        );

        _ImageProcPolicy::addImagePublisher( "intensity" );
        _ImageProcPolicy::addImagePublisher( "color" );
        _ImageProcPolicy::addImagePublisher( "orientation" );
#ifdef ENV_WITH_DYNAMIC_CHANNELS
        _ImageProcPolicy::addImagePublisher( "flicker" );
        _ImageProcPolicy::addImagePublisher( "motion" );
#endif
        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }

    QUICKDEV_DECLARE_IMAGE_CALLBACK( imageCB )
    {
        cv::Mat const & image = image_msg->image;

        IplImage ipl_image = image;

        env_dims indims;
        env_rgb_pixel * input = opencv_to_env_rgb_pixel( &ipl_image, &indims );

        env_image visual_cortex_out = env_img_initializer;
        env_image intensity = env_img_initializer;
        env_image color = env_img_initializer;
        env_image orientation = env_img_initializer;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
        env_image flicker = env_img_initializer;
        env_image motion = env_img_initializer;
#endif

        env_mt_visual_cortex_input
        (
            num_threads_,
            &visual_cortex_,
            &env_params_,
            "visualcortex",
            input,
            0,
            indims,
            NULL,
            NULL,
            &visual_cortex_out,
            &intensity,
            &color,
            &orientation
#ifdef ENV_WITH_DYNAMIC_CHANNELS
            ,
            &flicker,
            &motion
#endif
        );

        env_visual_cortex_rescale_ranges
        (
            &visual_cortex_out,
            &intensity,
            &color,
            &orientation
#ifdef ENV_WITH_DYNAMIC_CHANNELS
            ,
            &flicker,
            &motion
#endif
        );

        _ImageProcPolicy::publishImages
        (
            "saliency_image",    quickdev::opencv_conversion::fromIplImage( env_image_to_opencv( &visual_cortex_out ), "", "mono8" ),
            "intensity_image",   quickdev::opencv_conversion::fromIplImage( env_image_to_opencv( &intensity )        , "", "mono8" ),
            "color_image",       quickdev::opencv_conversion::fromIplImage( env_image_to_opencv( &color )            , "", "mono8" ),
            "orientation_image", quickdev::opencv_conversion::fromIplImage( env_image_to_opencv( &orientation )      , "", "mono8" )
#ifdef ENV_WITH_DYNAMIC_CHANNELS
            ,
            "flicker_image",     quickdev::opencv_conversion::fromIplImage( env_image_to_opencv( &flicker ), "", "mono8" ),
            "motion_image",      quickdev::opencv_conversion::fromIplImage( env_image_to_opencv( &motion ) , "", "mono8" )
#endif
        );
    }
};

#endif // SALIENCY_SALIENCYNODE_H_
