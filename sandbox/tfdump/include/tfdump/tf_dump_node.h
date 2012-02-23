/***************************************************************************
 *  include/tfdump/tf_dump_node.h
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

#ifndef TFDUMP_TFDUMPNODE_H_
#define TFDUMP_TFDUMPNODE_H_

#include <quickdev/node.h>
#include <quickdev/tf_tranceiver_policy.h>
#include <humanoid_recognizers/humanoid_recognizer_policy.h>

#include <iostream>
#include <ctime>
#include <fstream>

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;
typedef HumanoidRecognizerPolicy<> _HumanoidRecognizerPolicy;

QUICKDEV_DECLARE_NODE( TfDump, _TfTranceiverPolicy, _HumanoidRecognizerPolicy )

QUICKDEV_DECLARE_NODE_CLASS( TfDump )
{
    std::ofstream output_file_;
    std::string user_name_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( TfDump )
    {
        //
    }

    ~TfDumpNode()
    {
        output_file_.close();
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        user_name_ = ros::ParamReader<std::string, 1>::readParam( nh_rel, "user_name", "user1" );
        auto const output_path_prefix = ros::ParamReader<std::string, 1>::readParam( nh_rel, "output_path", "" );

        std::stringstream filename_ss;
        filename_ss << output_path_prefix << user_name_ << "_" << ros::Time::now() << ".csv";

        output_file_.open( filename_ss.str() );

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        QUICKDEV_LOCK_CACHE_AND_GET( states_cache_, states_msg );
        if( !states_msg ) return;

        _HumanoidRecognizerPolicy::updateHumanoids( states_msg );

        auto const & desired_user = _HumanoidRecognizerPolicy::lookupHumanoid( user_name_ );

        auto const now = ros::Time::now();

        for( auto joint = desired_user.cbegin(); joint != desired_user.cend(); ++joint )
        {
            output_file_
                << now << ", "
                << joint->name << ", "
                << joint->pose.confidence << ", "
                << joint->pose.pose.position.x << ", "
                << joint->pose.pose.position.y << ", "
                << joint->pose.pose.position.z << ", "
                << joint->pose.pose.orientation.x << ", "
                << joint->pose.pose.orientation.y << ", "
                << joint->pose.pose.orientation.z << ", "
                << joint->pose.pose.orientation.w << std::endl;
        }

        output_file_ << std::endl;
    }
};

#endif // TFDUMP_TFDUMPNODE_H_
