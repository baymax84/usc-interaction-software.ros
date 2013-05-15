/***************************************************************************
 *  include/quickdev/reconfigure_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_RECONFIGUREPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_RECONFIGUREPOLICY_H_

#include <quickdev/node_handle_policy.h>
#include <quickdev/auto_bind.h>
#include <dynamic_reconfigure/server.h>

namespace quickdev
{

QUICKDEV_DECLARE_POLICY( Reconfigure, NodeHandlePolicy )

template<class __ReconfigureType>
QUICKDEV_DECLARE_POLICY_CLASS( Reconfigure )
{
	QUICKDEV_MAKE_POLICY_FUNCS( Reconfigure )

public:
	typedef __ReconfigureType _ReconfigureType;
	typedef dynamic_reconfigure::Server<__ReconfigureType> _ReconfigureServer;
	typedef ReconfigurePolicy<__ReconfigureType> _ReconfigurePolicy;

protected:
	__ReconfigureType config_;

private:
	_ReconfigureServer * server_;
	typename _ReconfigureServer::CallbackType external_callback_;

public:
	QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( Reconfigure ),
		server_( NULL ),
		initialized_( false )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}

	QUICKDEV_ENABLE_INIT
	{
		printPolicyActionStart( "initialize", this );

		auto & nh_rel = NodeHandlePolicy::getNodeHandle();

		const std::string reconfigure_namespace_name( getMetaParamDef<std::string>( "reconfigure_namespace_param", "reconfigure_namespace", args... ) );
		const std::string reconfigure_namespace( ros::ParamReader<std::string, 1>::readParam( nh_rel, reconfigure_namespace_name, "reconfigure" ) );
		ros::NodeHandle reconfigure_nh( nh_rel, reconfigure_namespace );

		PRINT_INFO( "Creating dynamic reconfigure server on topic [%s]", reconfigure_nh.getNamespace().c_str() );
		server_ = new _ReconfigureServer( reconfigure_nh );

		server_->setCallback( quickdev::auto_bind( &_ReconfigurePolicy::reconfigureCB_0, this ) );

		QUICKDEV_SET_INITIALIZED();

		printPolicyActionDone( "initialize", this );
	}

	~ReconfigurePolicy()
	{
		if( server_ ) delete server_;
	}

	void registerCallback( const typename _ReconfigureServer::CallbackType & external_callback )
	{
		QUICKDEV_CHECK_INITIALIZED();

		external_callback_ = external_callback;
	}

private:
	QUICKDEV_DECLARE_RECONFIGURE_CALLBACK( reconfigureCB_0, __ReconfigureType )
	{
		config_ = config;
		if( external_callback_ ) external_callback_( config, level );
	}
};

}

#endif // QUICKDEVCPP_QUICKDEV_RECONFIGUREPOLICY_H_
