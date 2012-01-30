/***************************************************************************
 *  include/quickdev/time.h
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

#ifndef QUICKDEVCPP_QUICKDEV_TIME_H_
#define QUICKDEVCPP_QUICKDEV_TIME_H_

#include <quickdev/macros.h>
#include <chrono>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::time_point<Clock> Time;
typedef double DurationSeconds;
typedef std::chrono::duration<DurationSeconds> Duration;

inline Time now()
{
    return Time::clock::now();
}

class Timer
{
public:
    void reset();
    quickdev::Time const & start();
    DurationSeconds stop();
    DurationSeconds getDuration( quickdev::Time const & otherTime );
    DurationSeconds getDuration();
    DurationSeconds update();
    quickdev::Time const & now();
    quickdev::Time const & last();

private:
    quickdev::Time time_;
    quickdev::Time last_;
};

} // quickdev


#endif // QUICKDEVCPP_QUICKDEV_TIME_H_
