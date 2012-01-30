/***************************************************************************
 *  include/quickdev_examples/image_proc_policy.h
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

#ifndef QUICKDEV_QUICKDEVTESTS_IMAGEPROCPOLICY_H_
#define QUICKDEV_QUICKDEVTESTS_IMAGEPROCPOLICY_H_

#include <quickdev/node.h>
#include <quickdev/image_proc_policy.h>

QUICKDEV_DECLARE_NODE( ImageProcPolicy, quickdev::ImageProcPolicy )

QUICKDEV_DECLARE_NODE_CLASS( ImageProcPolicy )
{
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ImageProcPolicy )
    {

    }

    IMAGE_PROC_PROCESS_IMAGE( image_ptr )
    {
        cv::Mat image = image_ptr->image;

        // detect edges with cv::Canny and publish the resulting single-channel 8-bit image (need to specify mono8 encoding in fromMat() )
        cv::Mat bw_image;
        cv::cvtColor( image, bw_image, CV_BGR2GRAY );
        cv::Canny( bw_image, bw_image, 0.2, 0.4 );

        publishImages( "output_image", quickdev::opencv_conversion::fromMat( bw_image, "", "mono8" ) );
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // QUICKDEV_QUICKDEVTESTS_IMAGEPROCPOLICY_H_
