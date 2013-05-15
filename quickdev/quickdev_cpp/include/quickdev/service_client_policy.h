/***************************************************************************
 *  include/quickdev/service_client_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_SERVICECLIENTPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_SERVICECLIENTPOLICY_H_

#include <quickdev/node_handle_policy.h>
#include <ros/service_client.h>
#include <boost/thread/mutex.hpp>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

QUICKDEV_DECLARE_POLICY( ServiceClient, NodeHandlePolicy )

template<class __Service, unsigned int __Id__ = 0>
QUICKDEV_DECLARE_POLICY_CLASS( ServiceClient )
{
    QUICKDEV_MAKE_POLICY_FUNCS( ServiceClient )

public:
    typedef typename __Service::Request _ServiceRequest;
    typedef typename __Service::Response _ServiceResponse;

private:
    ros::ServiceClient client_;
    boost::mutex service_mutex_;

    std::string service_name_, service_topic_name_;
    bool is_valid_;

    static constexpr double _DEF_callService_wait_time = 2.0;
    static constexpr unsigned int _DEF_callService_attempts = 1;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( ServiceClient ),
        service_name_( "service" ),
        service_topic_name_( "service" ),
        is_valid_( false ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    QUICKDEV_ENABLE_INIT()
    {
        auto & nh_rel = NodeHandlePolicy::getNodeHandle();

        auto const enable_key_ids( getMetaParamDef<bool>( "enable_key_ids", false, std::forward<__Args>( args )... ) );

        service_name_ = policy::readPolicyParamAuto<std::string>( nh_rel, enable_key_ids, "service_name_param", __Id__, "service_name", "service", std::forward<__Args>( args )... );

        service_topic_name_ = ros::NodeHandle( nh_rel, service_name_ ).getNamespace();
        // now that we have a final value for the service name, create a service with that name

        QUICKDEV_SET_INITIALIZED();

        connectToService( true );
    }

    //! Attempt to connect to the service
    /*! - Print warnings if the connection attempt failed
     *  - If \param show_status_on_success and the attempt to connect succeeded, show that info
     *
     *  \return is_valid_, the state of the service connection */
    bool const & connectToService( bool const & show_status_on_success = false )
    {
        QUICKDEV_ASSERT_INITIALIZED( is_valid_ );

        ros::NodeHandle & nh_rel = NodeHandlePolicy::getNodeHandle();

        is_valid_ = client_.exists() && client_.isValid();
        if( !is_valid_ )
        {
            PRINT_INFO( "Attempting to connect to service [ %s ] on topic [ %s ]...", QUICKDEV_GET_SERVICE_NAME( __Service ).c_str(), service_topic_name_.c_str() );
            client_ = nh_rel.serviceClient<__Service>( service_name_, true );
            is_valid_ = client_.exists() && client_.isValid();

            if( is_valid_ ) PRINT_INFO( "Connected to service on topic [ %s ].", service_topic_name_.c_str() );
            else PRINT_WARN( "Could not connect to service on topic [ %s ]!", service_topic_name_.c_str() );
        }

        return is_valid_;
    }

    //! Attempt to call the service.
    /*! - If the service is not available, attempt to connect \param attempts times before aborting, waiting \param wait_time seconds in between each attempt
     *  - if \param attempts is 0, try indefinitely
     *
     *  \return false if aborted, otherwise return the result of the service call */

    bool callService( __Service & service, ros::Duration const & wait_time = ros::Duration( _DEF_callService_wait_time ), unsigned int attempts = _DEF_callService_attempts )
    {
        return callService( service.request, service.response, wait_time, attempts );
    }

    //! Expanded version of callService
    /*! Takes \param service.request and \param service.response as arguments instead of just \param service
        \return the result of the service call or false if the connection failed */
    bool callService( _ServiceRequest & request, _ServiceResponse & response, ros::Duration const & wait_time = ros::Duration( _DEF_callService_wait_time ), unsigned int attempts = _DEF_callService_attempts )
    {
        if( !service_mutex_.try_lock() ) return false;

        while( !connectToService() )
        {
            // if we're on the last attempt and we still haven't connected, abort
            if( attempts == 1 ) return false;

            if( attempts > 0 ) --attempts;

            if( attempts > 0 ) PRINT_WARN( "Will try %u more times before aborting.", attempts );
            else PRINT_WARN( "Will try indefinitely." );

            client_.waitForExistence( wait_time );
        }

        service_mutex_.unlock();

        return client_.call( request, response );
    }
};

}

#endif // QUICKDEVCPP_QUICKDEV_SERVICECLIENTPOLICY_H_
