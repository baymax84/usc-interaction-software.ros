#define __ActionServerPolicy ActionServerPolicy<__Action>

// =========================================================================================================================================
template<class __Action>
QUICKDEV_DECLARE_MESSAGE_CALLBACK( ActionServerPolicy<__Action>::executeActionCB, typename _GoalMsg )
{
    QUICKDEV_ASSERT_INITIALIZED();

    if( !action_server_ )
    {
        PRINT_ERROR( "Cannot send execute request to un-initialized server" );
        return;
    }

    _GoalMsgCallbackPolicy::invokeCallback( msg, action_server_ );
}

// =============================================================================================================================================
template<class __Action>
void __ActionServerPolicy::postInit()
{
    auto & nh_rel = NodeHandlePolicy::getNodeHandle();

    // we use simple_bind here to link the function required by 'server_' to the function defined by _FUNCTION_BASE_TYPE.
    // we need to pass f(x) to server_ but when server_ calls f(x), we actually want to call f(x,y)
    // so simple_bind takes f(x,y) and returns f(x); then, when f(x) is called, we automatically call f(x,y)
    action_server_( nh_rel, action_topic_name_, simple_bind( &ActionServerPolicy::executeActionCB, this ), false );
}

#undef __ActionServerPolicy
