/***************************************************************************
 *  include/quickdev/pixel.h
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

#ifndef QUICKDEVCPP_QUICKDEV_PIXEL_H_
#define QUICKDEVCPP_QUICKDEV_PIXEL_H_

#include <quickdev/feature.h>
#include <opencv/cv.h>
#include <vector>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

template<class __Data>
class Pixel : public quickdev::Feature<__Data>
{
public:
    template<class... __Args>
    Pixel( __Args&&... args ) : quickdev::Feature<__Data>( std::forward<__Args>( args )... )
    {
        //
    }
};

namespace pixel
{

template<class __Data, int __Dim__>
Pixel<__Data> make_pixel( cv::Vec<__Data, __Dim__> const & vec )
{
    std::vector<__Data> data( __Dim__ );
    std::copy( vec.val, vec.val + __Dim__, data.begin() );

    return Pixel<__Data>( data );
}

template<class __OutputData, class __InputData, int __Dim__>
Pixel<__OutputData> make_pixel( cv::Vec<__InputData, __Dim__> const & vec )
{
    std::vector<__OutputData> data( __Dim__ );
    std::copy( vec.val, vec.val + __Dim__, data.begin() );

    return Pixel<__OutputData>( data );
}

} // pixel

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_PIXEL_H_
