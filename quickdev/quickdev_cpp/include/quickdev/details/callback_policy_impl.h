#define __CallbackPolicy CallbackPolicy<__CallbackReturn( __CallbackArgs... )>

// =============================================================================================================================================
template<class __CallbackReturn, class... __CallbackArgs>
void __CallbackPolicy::registerCallback( _CallbackType const & callback )
{
    callbacks_.push_back( callback );
}

// =============================================================================================================================================
template<class __CallbackReturn, class... __CallbackArgs> template<class __Return>
QUICKDEV_ENABLE_IF_SAME( __CallbackReturn, __Return, void )
__CallbackPolicy::invokeCallback_0( __CallbackArgs&&... args ) const
{
    for( auto callback_it = callbacks_.begin(); callback_it != callbacks_.end(); ++callback_it )
    {
        if( *callback_it ) (*callback_it)( std::forward<__CallbackArgs>( args )... );
    }
}

// =============================================================================================================================================
template<class __CallbackReturn, class... __CallbackArgs> template<class __Return>
QUICKDEV_ENABLE_IF_NOT_SAME( __CallbackReturn, __Return, void )
__CallbackPolicy::invokeCallback_0( __CallbackArgs&&... args ) const
{
    __CallbackReturn default_return = __CallbackReturn();

    for( auto callback_it = callbacks_.begin(); callback_it != callbacks_.end(); ++callback_it )
    {
        if( *callback_it ) default_return = (*callback_it)( std::forward<__CallbackArgs>( args )... );
    }

    return default_return;
}

// =============================================================================================================================================
template<class __CallbackReturn, class... __CallbackArgs>
__CallbackReturn __CallbackPolicy::invokeCallback( __CallbackArgs&&... args ) const
{
    // in order to enable/disable functions with enable_if, they need to be directly dependent on some outer type
    return invokeCallback_0<__CallbackReturn>( std::forward<__CallbackArgs>( args )... );
}

#undef __CallbackPolicy
