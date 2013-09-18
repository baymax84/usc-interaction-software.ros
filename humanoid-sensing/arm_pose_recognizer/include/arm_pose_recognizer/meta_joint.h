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
#include <tf/LinearMath/Transform.h>

class MetaJoint
{
public:
    // parent end <- this start
    tf::Transform transform_to_parent_;
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

    MetaJoint( tf::Transform const & transform_to_parent, std::string const & name = "", std::string const & parent_name = "" )
    :
        // the parent transform needs rotation and the unit vector of translation
        // note that if the translation component is ( 0, 0, 0 ) then the normalized translation will be ( nan, nan, nan )
        // this is the desired behavior, as the referenced link has a length of 0, and should never be used
        transform_to_parent_( transform_to_parent.getRotation(), transform_to_parent.getOrigin().normalized() ),
        name_( name ),
        parent_name_( parent_name )
    {
        tf::Vector3 const & vec_to_parent = transform_to_parent_.getOrigin();
        printf( "Created meta-joint [ %s ] with parent [ %s ]\n", name_.c_str(), parent_name_.c_str() );
        printf( "and transform [ %f ][ %f ][ %f ]\n", vec_to_parent.x(), vec_to_parent.y(), vec_to_parent.z() );
    }
};

#endif // ARMPOSERECOGNIZER_METAJOINT_H_
