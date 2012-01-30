/***************************************************************************
 *  src/time.cpp
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

#include <quickdev/time.h>

// ######################################################################
void QUICKDEV_GET_INTERNAL_NAMESPACE()::Timer::reset()
{
    last_ = time_;
    time_ = quickdev::now();
}

// ######################################################################
quickdev::Time const & QUICKDEV_GET_INTERNAL_NAMESPACE()::Timer::start()
{
    last_ = time_;
    time_ = quickdev::now();
    return time_;
}

// ######################################################################
quickdev::DurationSeconds QUICKDEV_GET_INTERNAL_NAMESPACE()::Timer::stop()
{
    double const dt = getDuration();
    reset();
    return dt;
}

quickdev::DurationSeconds QUICKDEV_GET_INTERNAL_NAMESPACE()::Timer::getDuration( quickdev::Time const & otherTime )
{
    return (quickdev::DurationSeconds) std::chrono::duration_cast<std::chrono::microseconds>( otherTime - time_ ).count() / 1000000.0;
}

quickdev::DurationSeconds QUICKDEV_GET_INTERNAL_NAMESPACE()::Timer::getDuration()
{
    return getDuration( quickdev::now() );
}

// ######################################################################
quickdev::DurationSeconds QUICKDEV_GET_INTERNAL_NAMESPACE()::Timer::update()
{
    auto const now = quickdev::now();
    auto const dt = getDuration( now );
    last_ = time_;
    time_ = now;
    return dt;
}

quickdev::Time QUICKEV_GET_INTERNAL_NAMESPACE()::Timer::now()
{
    return time_;
}

quickdev::Time QUICKEV_GET_INTERNAL_NAMESPACE()::Timer::last()
{
    return last_;
}
