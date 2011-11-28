/***************************************************************************
 *  include/quickdev/node_handle_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_NODEHANDLEPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_NODEHANDLEPOLICY_H_

#include <quickdev/param_reader.h>
#include <quickdev/policy.h>
#include <ros/node_handle.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

QUICKDEV_DECLARE_POLICY( NodeHandle, Policy )

QUICKDEV_DECLARE_POLICY_CLASS( NodeHandle )
{
    QUICKDEV_MAKE_POLICY_FUNCS( NodeHandle )

private:
    ros::NodeHandle nh_rel_;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( NodeHandle ),
        nh_rel_( getFirstOfType<ros::NodeHandle>( args... ) )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    ros::NodeHandle & getNodeHandle()
    {
        return nh_rel_;
    }
};

}

#endif // QUICKDEVCPP_QUICKDEV_NODEHANDLEPOLICY_H_
