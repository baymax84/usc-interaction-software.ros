/***************************************************************************
 *  include/quickdev/threading.h
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

#ifndef QUICKDEVCPP_QUICKDEV_THREADING_H_
#define QUICKDEVCPP_QUICKDEV_THREADING_H_

#include <quickdev/macros.h>
#include <quickdev/console.h>
#include <boost/thread/mutex.hpp>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{
    // =========================================================================================================================================
    class Mutex
    {
    public:
        boost::mutex mutex_;
        typedef boost::unique_lock<boost::mutex> _UniqueLock;

        _UniqueLock tryLock()
        {
            return _UniqueLock( mutex_, boost::try_to_lock_t() );
        }

        _UniqueLock lock()
        {
            return _UniqueLock( mutex_ );
        }
    };

    // =========================================================================================================================================
    template<class __Storage>
    class MutexedCache : public Mutex
    {
    public:
        __Storage cache_;

        MutexedCache( __Storage const & cache = __Storage() )
        :
            cache_( cache )
        {
            //
        }

        _UniqueLock tryLockAndUpdate( __Storage const & cache )
        {
            auto lock = tryLock();
            if( lock ) cache_ = cache;
            return lock;
        }

        MutexedCache & operator=( __Storage const & cache )
        {
            auto lock = tryLock();
            if( lock ) cache_ = cache;
            return *this;
        }

        const __Storage & get() const
        {
            return cache_;
        }

        __Storage & get()
        {
            return cache_;
        }
    };

    // =========================================================================================================================================
    template<class __Message>
    class MessageCache : public MutexedCache<typename __Message::ConstPtr>{};

    template<>
    class MessageCache<void>{};

    // =========================================================================================================================================
    template
    <
        class __FuncReturn,
        class... __FuncArgs,
        typename std::enable_if<(!std::is_same<__FuncReturn, void>::value), int>::type = 0
    >
    std::pair<bool, __FuncReturn> tryFunc( __QUICKDEV_FUNCTION_TYPE<__FuncReturn( __FuncArgs&&... )> func, size_t const & max_attempts, long unsigned int const & sleep_time )
    {
        size_t attempts = 0;
        do
        {
            try
            {
                return std::make_pair( true, func() );
            }
            catch( std::exception const & ex )
            {
                PRINT_ERROR( "%s", ex.what() );
                PRINT_WARN( "Attempt %zu of %zu failed; sleeping for %f seconds", attempts + 1, max_attempts, sleep_time / 1000000.0 );
                if( max_attempts > 0 ) attempts ++;
                usleep( sleep_time );
            }
        }
        while( attempts < max_attempts || max_attempts == 0 );

        return std::make_pair( false, __FuncReturn() );
    }

    // =========================================================================================================================================
    template
    <
        class __FuncReturn,
        class... __FuncArgs,
        typename std::enable_if<(std::is_same<__FuncReturn, void>::value), int>::type = 0
    >
    bool tryFunc( __QUICKDEV_FUNCTION_TYPE<__FuncReturn( __FuncArgs&&... )> func, size_t const & max_attempts, long unsigned int const & sleep_time )
    {
        size_t attempts = 0;
        do
        {
            try
            {
                func();
                return true;
            }
            catch( std::exception const & ex )
            {
                PRINT_ERROR( "%s", ex.what() );
                PRINT_WARN( "Attempt %zu of %zu failed; sleeping for %f seconds", attempts + 1, max_attempts, sleep_time / 1000000.0 );
                if( max_attempts > 0 ) attempts ++;
                usleep( sleep_time );
            }
        }
        while( attempts < max_attempts || max_attempts == 0 );

        return false;
    }

    // =========================================================================================================================================
    template
    <
        class __FuncReturn,
        class... __FuncArgs
    >
    auto tryFunc( __QUICKDEV_FUNCTION_TYPE<__FuncReturn( __FuncArgs&&... )> func, size_t const & max_attempts ) -> decltype( tryFunc( func, max_attempts, 1000 ) )
    {
        return tryFunc( func, max_attempts, 1000 );
    }

    // =========================================================================================================================================
    template
    <
        class __FuncReturn,
        class... __FuncArgs
    >
    auto tryFunc( __QUICKDEV_FUNCTION_TYPE<__FuncReturn( __FuncArgs&&... )> func ) -> decltype( tryFunc( func, 1 ) )
    {
        return tryFunc( func, 1 );
    }
}

#endif // QUICKDEVCPP_QUICKDEV_THREADING_H_
