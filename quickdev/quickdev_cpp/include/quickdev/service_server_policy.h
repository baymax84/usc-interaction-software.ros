/***************************************************************************
 *  include/quickdev/service_server_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_SERVICESERVERPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_SERVICESERVERPOLICY_H_

#include <quickdev/node_handle_policy.h>
#include <quickdev/callback_policy.h>
#include <quickdev/auto_bind.h>
#include <ros/service_server.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

template<class __Service>
QUICKDEV_DECLARE_POLICY2( ServiceServer, NodeHandlePolicy, ServiceCallbackPolicy<__Service> )

template<class __Service, unsigned int __Id__ = 0>
QUICKDEV_DECLARE_POLICY_CLASS2( ServiceServer, __Service )
{
    QUICKDEV_MAKE_POLICY_FUNCS( ServiceServer )

protected:
    typedef typename __Service::Request _ServiceRequest;
    typedef typename __Service::Response _ServiceResponse;
    typedef ServiceServerPolicy<__Service, __Id__> _ServiceServerPolicy;
    typedef ServiceCallbackPolicy<__Service> _ServiceCallbackPolicy;

    ros::ServiceServer server_;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR2( ServiceServer, __Service ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    QUICKDEV_ENABLE_INIT()
    {
        auto & nh_rel = NodeHandlePolicy::getNodeHandle();

        bool const enable_key_ids( getMetaParamDef<bool>( "enable_key_ids", false, args... ) );

        auto const service_name = policy::readPolicyParamAuto<std::string>( nh_rel, enable_key_ids, "service_name_param", __Id__, "service_name", "service", args... );

        ros::NodeHandle service_nh( nh_rel, service_name );
        PRINT_INFO( "Creating service server [%s] on topic [%s]", ros::service_traits::DataType<__Service>::value(), service_nh.getNamespace().c_str() );

        server_ = nh_rel.advertiseService( service_name, &_ServiceServerPolicy::serviceCB, this );

        QUICKDEV_SET_INITIALIZED();
    }

private:
    QUICKDEV_DECLARE_SERVICE_CALLBACK( serviceCB, typename __Service )
    {
        QUICKDEV_ASSERT_INITIALIZED( false );

        return _ServiceCallbackPolicy::invokeCallback( request, response );
    }
};

}

#endif // QUICKDEVCPP_QUICKDEV_SERVICESERVERPOLICY_H_
