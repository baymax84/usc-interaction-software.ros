/***************************************************************************
 *  include/quickdev_examples/service_client_policy.h
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

#ifndef QUICKDEV_QUICKDEVTESTS_SERVICECLIENTPOLICY_H_
#define QUICKDEV_QUICKDEVTESTS_SERVICECLIENTPOLICY_H_

#include <quickdev/node.h>
#include <quickdev/service_client_policy.h>
#include <quickdev_examples/TestService1.h>
#include <quickdev_examples/TestService2.h>

typedef quickdev_examples::TestService1 _Service1;
typedef quickdev::ServiceClientPolicy<_Service1> _ServiceClientPolicy1;
typedef quickdev_examples::TestService2 _Service2;
typedef quickdev::ServiceClientPolicy<_Service2> _ServiceClientPolicy2;

QUICKDEV_DECLARE_NODE( ServiceClientPolicy, _ServiceClientPolicy1, _ServiceClientPolicy2 )

QUICKDEV_DECLARE_NODE_CLASS( ServiceClientPolicy )
{
private:
	bool mode_;

	QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ServiceClientPolicy ),
		mode_( false )
	{}

public:
	QUICKDEV_SPIN_FIRST()
	{
		//initAll();

		_ServiceClientPolicy1::init( "service_name_param", std::string( "service1_name" ) );
		_ServiceClientPolicy2::init( "service_name_param", std::string( "service2_name" ) );
	}

	QUICKDEV_SPIN_ONCE()
	{
		PRINT_INFO( "Spinning..." );

		if( mode_ )
		{
			_Service1 service;
			_ServiceClientPolicy1::callService( service );
		}
		else
		{
			_Service2 service;
			_ServiceClientPolicy2::callService( service, ros::Duration( 2.0 ), 2 );
		}

		mode_ = !mode_;
	}
};

#endif // QUICKDEV_QUICKDEVTESTS_SERVICECLIENTPOLICY_H_
