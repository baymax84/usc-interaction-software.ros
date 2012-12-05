/***************************************************************************
 *  include/arm_pose_recognizer/meta_joint.h
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
 *  * Neither the name of humanoid-sensing nor the names of its
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

#ifndef ARMPOSERECOGNIZER_METAJOINT_H_
#define ARMPOSERECOGNIZER_METAJOINT_H_

#include <map>
#include <string>
#include <LinearMath/btTransform.h>

class MetaJoint
{
public:
    btQuaternion rotation_;
    btTransform relative_transform_;
    std::string name_;
    std::string parent_name_;

    MetaJoint( MetaJoint const & other )
    :
        rotation_( other.rotation_ ),
        relative_transform_( other.relative_transform_ ),
        name_( other.name_ ),
        parent_name_( other.parent_name_ )
    {
        //
    }

    MetaJoint( btQuaternion const & rotation = btQuaternion( 0, 0, 0, 1 ), std::string const & name = "", std::string const & parent_name = "" )
    :
        rotation_( rotation ),
        relative_transform_( rotation, btVector3( 1, 0, 0 ) ),
        name_( name ),
        parent_name_( parent_name )
    {
        //
    }
};

#endif // ARMPOSERECOGNIZER_METAJOINT_H_
