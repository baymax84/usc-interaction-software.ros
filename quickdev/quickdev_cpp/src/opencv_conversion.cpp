/***************************************************************************
 *  src/opencv_conversion.cpp
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
 *  * Neither the name of seabee3-ros-pkg nor the names of its
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

#include <quickdev/opencv_conversion.h>

// =============================================================================================================================================
sensor_msgs::Image::Ptr QUICKDEV_GET_INTERNAL_NAMESPACE()::opencv_conversion::fromMat( cv::Mat const & mat, std::string const & frame_id, std::string const & encoding )
{
    cv_bridge::CvImage image_wrapper;

    image_wrapper.image = mat;
    image_wrapper.encoding = encoding;
    image_wrapper.header.frame_id = frame_id;
    image_wrapper.header.stamp = ros::Time::now();

    return image_wrapper.toImageMsg();
}

// =============================================================================================================================================
sensor_msgs::Image::Ptr QUICKDEV_GET_INTERNAL_NAMESPACE()::opencv_conversion::fromIplImage( IplImage * image_ptr, std::string const & frame_id )
{
    //cv_bridge::CvImage image_wrapper;

    //image_wrapper.image = cv::Mat( image_ptr );
    //image_wrapper.encoding = "bgr8";
    //image_wrapper.frame_name = frame_id;

    //return image_wrapper;

    auto result = sensor_msgs::CvBridge::cvToImgMsg( image_ptr );
    result->header.frame_id = frame_id;
    result->header.stamp = ros::Time::now();
    return result;
}

// =============================================================================================================================================
cv_bridge::CvImageConstPtr QUICKDEV_GET_INTERNAL_NAMESPACE()::opencv_conversion::fromImageMsg( sensor_msgs::Image::ConstPtr const & image_msg )
{
    cv_bridge::CvImageConstPtr cv_image_ptr;

    try
    {
        cv_image_ptr = cv_bridge::toCvShare( image_msg );
    }
    catch (cv_bridge::Exception& e)
    {
        PRINT_ERROR( "cv_bridge exception: %s", e.what() );
    }

    return cv_image_ptr;
}

// =============================================================================================================================================
cv_bridge::CvImageConstPtr QUICKDEV_GET_INTERNAL_NAMESPACE()::opencv_conversion::fromImageMsg( sensor_msgs::Image const & image_msg )
{
    return fromImageMsg( make_const_shared( image_msg ) );
}
