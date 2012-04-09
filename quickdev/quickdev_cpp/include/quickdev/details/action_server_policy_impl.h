#define __ActionServerPolicy ActionServerPolicy<__Action, __Id__>

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
QUICKDEV_DECLARE_INIT( ActionServerPolicy<__Action, __Id__>:: )
{
    auto & nh_rel = NodeHandlePolicy::getNodeHandle();

    auto const enable_key_ids( getMetaParamDef<bool>( "enable_key_ids", false, std::forward<__Args>( args )... ) );

    auto const action_name = policy::readPolicyParamAuto<std::string>( nh_rel, enable_key_ids, "action_name_param", __Id__, "action_name", "action", std::forward<__Args>( args )... );

    ros::NodeHandle action_nh( nh_rel, action_name );
    PRINT_INFO( "Creating action server [%s] on topic [%s]", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_nh.getNamespace().c_str() );

    action_server_ = new _ActionServer( nh_rel, action_name, auto_bind( &ActionServerPolicy::executeActionCB, this ), false );

    action_server_->start();

    QUICKDEV_SET_INITIALIZED();
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
QUICKDEV_DECLARE_MESSAGE_CALLBACK( (ActionServerPolicy<__Action, __Id__>::executeActionCB), typename _GoalMsg )
{
    QUICKDEV_ASSERT_INITIALIZED();

    PRINT_INFO( "Got goal callback" );

    if( !action_server_ )
    {
        PRINT_ERROR( "Cannot send execute request to un-initialized server" );
        return;
    }

    _GoalMsgCallbackPolicy::invokeCallback( msg, action_server_ );
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionServerPolicy::setInterrupted( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    action_server_->setPreempted( std::forward<__Args>( args )... );
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionServerPolicy::sendFeedback( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    action_server_->publishFeedback( std::forward<__Args>( args )... );
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionServerPolicy::setCompleted( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    action_server_->setSucceeded( std::forward<__Args>( args )... );
}

#undef __ActionServerPolicy
