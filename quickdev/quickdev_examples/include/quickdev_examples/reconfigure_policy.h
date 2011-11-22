/***************************************************************************
 *  include/quickdev_examples/reconfigure_policy.h
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

#ifndef QUICKDEV_QUICKDEVTESTS_RECONFIGUREPOLICY_H_
#define QUICKDEV_QUICKDEVTESTS_RECONFIGUREPOLICY_H_

#include <quickdev/node.h>
#include <quickdev/reconfigure_policy.h>
#include <quickdev_examples/TestConfig.h>
#include <quickdev_examples/Test2Config.h>

typedef quickdev_examples::TestConfig _Config1;
typedef quickdev_examples::Test2Config _Config2;
typedef quickdev::ReconfigurePolicy<_Config1> _ReconfigurePolicy1;
typedef quickdev::ReconfigurePolicy<_Config2> _ReconfigurePolicy2;

QUICKDEV_DECLARE_NODE( ReconfigurePolicy, _ReconfigurePolicy1, _ReconfigurePolicy2 )

QUICKDEV_DECLARE_NODE_CLASS( ReconfigurePolicy )
{
	QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ReconfigurePolicy )
	{
		//
	}

	QUICKDEV_SPIN_FIRST
	{
		/*! call init() on all initable policies
		 *  initAll(); */

		/*! call init( ... ) on all initable policies
		 *  initAll( "reconfigure_namespace_name", std::string( "reconfigure_namespace1" ) ); */

		/*! call init on just _ConfigReconfigurePolicy
		 *  All ReconfigurePolicy instances use getMetaParamDef to look up the key "reconfigure_namespace_param" in the list of init() params
		 *  and default to "reconfigure_namspace" if no matching key is found. This value is then fed into a ParamReader as the name of a param.
		 *  The value of that param is then used as the reconfigure namespace.
		 *
		 *  So, when the following line is called, a ParamReader will try to find "node_name/reconfigure_namespace1" or default to "reconfigure" (let's call the result <reconfigure_ns>)
		 *  Then a dynamic reconfigure server will be set up as: node_name/<reconfigure_ns>
		 *  If this function was given no arguments, the ParamReader would instead try to find "node_name/reconfigure_namespace" (since the default value for they meta-param key
		 *  "reconfigure_namespace_param", as defined in ReconfigurePolicy, is "reconfigure_namespace") or default to "reconfigure", etc */

		_ReconfigurePolicy1::getNodeHandle().setParam( "reconfigure_namespace1", "reconfigure1" );
		_ReconfigurePolicy1::init( "reconfigure_namespace_param", std::string( "reconfigure_namespace1" ) );
		_ReconfigurePolicy1::registerCallback( quickdev::auto_bind( &ReconfigurePolicyNode::reconfigureCB1, this ) );

		/// call init on just _2ConfigReconfigurePolicy
		_ReconfigurePolicy2::getNodeHandle().setParam( "reconfigure_namespace2", "reconfigure2" );
		_ReconfigurePolicy2::init( "reconfigure_namespace_param", std::string( "reconfigure_namespace2" ) );
		_ReconfigurePolicy2::registerCallback( quickdev::auto_bind( &ReconfigurePolicyNode::reconfigureCB2, this ) );

		/*! so now, when we run this node using the above inits, we can do:
		 *  ./node_name _reconfigure_namespace1:=reconfigure1 _reconfigure_namespace2:=reconfigure2
		 *  and our two dynamic reconfigure servers will work in harmony; the server for Config
		 *  will show up on /node_name/reconfigure1 and the server for Config2 will show up on
		 *  /node_name/reconfigure2 */
	}

	QUICKDEV_SPIN_ONCE
	{
		std::cout << "." << std::flush;
	}

	QUICKDEV_DECLARE_RECONFIGURE_CALLBACK( reconfigureCB1, _Config1 )
	{
		printf( "Got reconfigureCB1\n" );
	}

	QUICKDEV_DECLARE_RECONFIGURE_CALLBACK( reconfigureCB2, _Config2 )
	{
		printf( "Got reconfigureCB2\n" );
	}
};

#endif // QUICKDEV_QUICKDEVTESTS_RECONFIGUREPOLICY_H_
