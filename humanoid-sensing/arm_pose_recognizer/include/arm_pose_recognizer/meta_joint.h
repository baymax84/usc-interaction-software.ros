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
    // parent end <- this start
    btTransform transform_to_parent_;
    // this name
    std::string name_;
    // parent name
    std::string parent_name_;

    MetaJoint()
    {
        //
    }

    MetaJoint( MetaJoint const & other )
    :
        transform_to_parent_( other.transform_to_parent_ ),
        name_( other.name_ ),
        parent_name_( other.parent_name_ )
    {
        //
    }

    MetaJoint( btTransform const & transform_to_parent, std::string const & name = "", std::string const & parent_name = "" )
    :
        // the parent transform needs rotation and the unit vector of translation
        transform_to_parent_( transform_to_parent.getRotation(), transform_to_parent.getOrigin().normalized() ),
        name_( name ),
        parent_name_( parent_name )
    {
        //
    }
};

#endif // ARMPOSERECOGNIZER_METAJOINT_H_
