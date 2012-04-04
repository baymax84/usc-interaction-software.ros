/***************************************************************************
 *  include/quickdev/geometry_message_conversions.h
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

#ifndef QUICKDEVCPP_QUICKDEV_GEOMETRYMESSAGECONVERSIONS_H_
#define QUICKDEVCPP_QUICKDEV_GEOMETRYMESSAGECONVERSIONS_H_

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <LinearMath/btTransform.h>
#include <quickdev/unit.h>

typedef btVector3 _Vector3;
typedef btQuaternion _Quaternion;
typedef btTransform _Transform;

typedef geometry_msgs::Point _PointMsg;
typedef geometry_msgs::Vector3 _Vector3Msg;
typedef geometry_msgs::Twist _TwistMsg;
typedef geometry_msgs::Quaternion _QuaternionMsg;

// Vector3Msg -> *
DECLARE_UNIT_CONVERSION_LAMBDA( _Vector3Msg, _Vector3, msg, return _Vector3( msg.x, msg.y, msg.z ); )
DECLARE_UNIT_CONVERSION_LAMBDA( _Vector3Msg, _PointMsg, vec, _PointMsg msg; msg.x = vec.x; msg.y = vec.y; msg.z = vec.z; return msg; )
DECLARE_UNIT_CONVERSION_LAMBDA( _Vector3Msg, _Quaternion, msg, return _Quaternion( msg.z, msg.y, msg.x ); )

// Vector3 -> *
DECLARE_UNIT_CONVERSION_LAMBDA( _Vector3, _Vector3Msg, vec, _Vector3Msg msg; msg.x = vec.getX(); msg.y = vec.getY(); msg.z = vec.getZ(); return msg; )
DECLARE_UNIT_CONVERSION_LAMBDA( _Vector3, _PointMsg, vec, _PointMsg msg; msg.x = vec.getX(); msg.y = vec.getY(); msg.z = vec.getZ(); return msg; )
DECLARE_UNIT_CONVERSION_LAMBDA( _Vector3, _Quaternion, vec, return _Quaternion( vec.getX(), vec.getY(), vec.getZ() ); )

// PointMsg -> *
DECLARE_UNIT_CONVERSION_LAMBDA( _PointMsg, _Vector3Msg, point, _Vector3Msg msg; msg.x = point.x; msg.y = point.y; msg.z = point.z; return msg; )
DECLARE_UNIT_CONVERSION_LAMBDA( _PointMsg, _Vector3, point, return _Vector3( point.x, point.y, point.z ); )

// Quaternion -> *
DECLARE_UNIT_CONVERSION_LAMBDA( _Quaternion, _Vector3Msg, quat, _Vector3Msg msg; const btMatrix3x3 rot_mat( quat ); rot_mat.getEulerYPR( msg.z, msg.y, msg.x ); return msg; )
DECLARE_UNIT_CONVERSION_LAMBDA( _Quaternion, _Vector3, quat, double x, y, z; btMatrix3x3 const rot_mat( quat ); rot_mat.getEulerYPR( z, y, x ); return _Vector3( x, y, z ); )
DECLARE_UNIT_CONVERSION_LAMBDA( _Quaternion, _QuaternionMsg, quat, _QuaternionMsg msg; msg.x = quat.getX(); msg.y = quat.getY(); msg.z = quat.getZ(); msg.w = quat.getW(); return msg; )

// QuaternionMsg -> *
DECLARE_UNIT_CONVERSION_LAMBDA( _QuaternionMsg, _Quaternion, msg, return _Quaternion( msg.x, msg.y, msg.z, msg.w ); )

// Transform -> *
DECLARE_UNIT_CONVERSION_LAMBDA( _Transform, _TwistMsg, tf, _TwistMsg res; res.angular = unit::make_unit( tf.getRotation() ); res.linear = unit::make_unit( tf.getOrigin() ); return res; )

// TwistMsg -> *
DECLARE_UNIT_CONVERSION_LAMBDA( _TwistMsg, _Transform, twist, const _Quaternion quat( unit::convert<_Quaternion>( twist.angular ).normalized() ); _Vector3 const vec = unit::make_unit( twist.linear ); return _Transform( quat, vec ); )

// Transform *= Scalar
static void operator*=( _Transform & transform, double const & scale )
{
    _Vector3 angle_ypr = unit::make_unit( unit::convert<_Vector3Msg>( transform.getRotation() ) );
    angle_ypr *= scale;
    transform.setRotation( unit::convert<_Quaternion>( unit::convert<_Vector3Msg>( angle_ypr ) ) );
    transform.getOrigin() *= scale;
}


#endif // QUICKDEVCPP_QUICKDEV_GEOMETRYMESSAGECONVERSIONS_H_
