/***************************************************************************
 *  include/quickdev/numeric_unit_conversions.h
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

#ifndef QUICKDEVCPP_QUICKDEV_NUMERICUNITCONVERSIONS_H_
#define QUICKDEVCPP_QUICKDEV_NUMERICUNITCONVERSIONS_H_

#include <quickdev/unit.h>

DECLARE_NUMERIC_UNIT( Degree, double );
DECLARE_NUMERIC_UNIT( Radian, double );

DECLARE_SCALAR_UNIT_CONVERSION( Degree, Radian, M_PI / 180.0 );

DECLARE_NUMERIC_UNIT( Meter, double );
DECLARE_NUMERIC_UNIT( Foot, double );
DECLARE_NUMERIC_UNIT( Inch, double );

DECLARE_SCALAR_UNIT_CONVERSION( Foot, Meter, 3.2808399 );
DECLARE_SCALAR_UNIT_CONVERSION( Inch, Foot, 12 );
DECLARE_SCALAR_UNIT_CONVERSION( Inch, Meter, 12 * 3.2808399 );

DECLARE_NUMERIC_UNIT( Second, double );
DECLARE_NUMERIC_UNIT( Minute, double );
DECLARE_NUMERIC_UNIT( Hour, double );
DECLARE_NUMERIC_UNIT( Day, double );

DECLARE_SCALAR_UNIT_CONVERSION( Second, Minute, 60 );
DECLARE_SCALAR_UNIT_CONVERSION( Minute, Hour, 60 );
DECLARE_SCALAR_UNIT_CONVERSION( Hour, Day, 24 );
DECLARE_SCALAR_UNIT_CONVERSION( Second, Hour, 60 * 60 );
DECLARE_SCALAR_UNIT_CONVERSION( Second, Day, 60 * 60 * 24 );
DECLARE_SCALAR_UNIT_CONVERSION( Minute, Day, 60 * 24 );

#endif // QUICKDEVCPP_QUICKDEV_NUMERICUNITCONVERSIONS_H_
