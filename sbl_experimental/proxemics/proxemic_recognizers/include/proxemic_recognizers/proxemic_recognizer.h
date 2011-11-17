/***************************************************************************
 *  include/proxemic_recognizers/proxemic_recognizer.h
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

#ifndef PROXEMICRECOGNIZERS_PROXEMICRECOGNIZERS_PROXEMICRECOGNIZER_H_
#define PROXEMICRECOGNIZERS_PROXEMICRECOGNIZERS_PROXEMICRECOGNIZER_H_

#include <quickdev/node.h>

#include <humanoid_recognizers/humanoid_recognizer_policy.h>

typedef HumanoidRecognizerPolicy _HumanoidRecognizerPolicy;
QUICKDEV_DECLARE_NODE( ProxemicRecognizer, _HumanoidRecognizerPolicy )

typedef _HumanoidRecognizerPolicy::_HumanoidStateArrayMsg _HumanoidStateArrayMsg;
typedef _HumanoidRecognizerPolicy::_MarkerArrayMsg _MarkerArrayMsg;

QUICKDEV_DECLARE_NODE_CLASS( ProxemicRecognizer )
{
	QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ProxemicRecognizer )
	{
		//
	}

	QUICKDEV_SPIN_FIRST
	{
		initAll();
	}

	QUICKDEV_SPIN_ONCE
	{
		QUICKDEV_LOCK_CACHE_AND_GET( states_cache_, states_msg );
		if( !states_msg ) return;

		_MarkerArrayMsg markers;

		for( auto humanoid = states_msg->states.begin(); humanoid != states_msg->states.end(); ++humanoid )
		{
			//
		}

		_HumanoidRecognizerPolicy::update( markers );
	}
};

#endif // PROXEMICRECOGNIZERS_PROXEMICRECOGNIZERS_PROXEMICRECOGNIZER_H_
