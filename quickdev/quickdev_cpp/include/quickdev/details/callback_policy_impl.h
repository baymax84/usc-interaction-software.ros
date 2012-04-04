#define __CallbackPolicy CallbackPolicy<__CallbackReturn( __CallbackArgs... )>

// =============================================================================================================================================
template<class __CallbackReturn, class... __CallbackArgs>
void __CallbackPolicy::registerCallback( _CallbackType const & external_callback )
{
    external_callback_ = external_callback;
}

// =============================================================================================================================================
template<class __CallbackReturn, class... __CallbackArgs> template<class __Return>
QUICKDEV_ENABLE_IF_SAME( __CallbackReturn, __Return, void )
__CallbackPolicy::invokeCallback_0( __CallbackArgs&&... args ) const
{
    if( external_callback_ ) external_callback_( std::forward<__CallbackArgs>( args )... );
}

// =============================================================================================================================================
template<class __CallbackReturn, class... __CallbackArgs> template<class __Return>
QUICKDEV_ENABLE_IF_NOT_SAME( __CallbackReturn, __Return, void )
__CallbackPolicy::invokeCallback_0( __CallbackArgs&&... args ) const
{
    static __CallbackReturn const default_return = __CallbackReturn();

    if( !external_callback_ ) return default_return;
    return external_callback_( std::forward<__CallbackArgs>( args )... );
}

// =============================================================================================================================================
template<class __CallbackReturn, class... __CallbackArgs>
__CallbackReturn __CallbackPolicy::invokeCallback( __CallbackArgs&&... args ) const
{
    // in order to enable/disable functions with enable_if, they need to be directly dependent on some outer type
    return invokeCallback_0<__CallbackReturn>( std::forward<__CallbackArgs>( args )... );
}

#undef __CallbackPolicy
