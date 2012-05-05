#define __ActionServerPolicy ActionServerPolicy<__Action, __Id__>

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
QUICKDEV_DECLARE_INIT( ActionServerPolicy<__Action, __Id__>:: )
{
    auto & nh_rel = NodeHandlePolicy::getNodeHandle();

    auto const enable_key_ids( getMetaParamDef<bool>( "enable_key_ids", false, std::forward<__Args>( args )... ) );

    auto const action_name = policy::readPolicyParamAuto<std::string>( nh_rel, enable_key_ids, "action_name_param", __Id__, "action_name", "action", std::forward<__Args>( args )... );

    ros::NodeHandle action_nh( nh_rel, action_name );

    action_topic_name_ = ros::NodeHandle( nh_rel, action_name ).getNamespace();

    PRINT_INFO( "Creating action server [%s] on topic [%s]", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );

    action_server_ = new _ActionServer( nh_rel, action_name, auto_bind( &ActionServerPolicy::executeActionCB, this ), false );
    action_server_->registerPreemptCallback( auto_bind( &ActionServerPolicy::preemptCB, this ) );
    action_server_->registerGoalCallback( auto_bind( &ActionServerPolicy::goalCB, this ) );

    action_server_->start();

    QUICKDEV_SET_INITIALIZED();
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
QUICKDEV_DECLARE_MESSAGE_CALLBACK( (ActionServerPolicy<__Action, __Id__>::executeActionCB), typename _GoalMsg )
{
    QUICKDEV_ASSERT_INITIALIZED();

    PRINT_INFO( "Got goal callback [%s] on topic [%s]", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );

    if( !action_server_ )
    {
        PRINT_ERROR( "Cannot send execute request to un-initialized server" );
        return;
    }

    // lock the goal mutex; the execute callback can freeze itself by locking this again
    {
        auto action_lock = quickdev::make_unique_lock( action_mutex_ );

        goal_msg_ = *msg;

        preempt_accepted_ = false;
        goal_initialized_ = true;
        result_msg_ = _ResultMsg();
        result_type_ = _GoalStatusMsg::ABORTED;
        result_info_ = "";

        // work gets done here; the current context is guaranteed to be a separate thread, so it's safe to block here
        _ExecuteCallbackPolicy::invokeCallback( msg, action_server_ );

        switch( result_type_ )
        {
        case _GoalStatusMsg::SUCCEEDED:
            action_server_->setSucceeded( result_msg_, result_info_ );
            break;
        case _GoalStatusMsg::ABORTED:
            action_server_->setAborted( result_msg_, result_info_ );
            break;
        case _GoalStatusMsg::PREEMPTED:
            action_server_->setPreempted( result_msg_, result_info_ );
            break;
        }
    }
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionServerPolicy::preemptCB()
{
    if( preempt_callback_ ) preempt_callback_( action_server_ );
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionServerPolicy::goalCB()
{
    if( goal_callback_ ) goal_callback_( action_server_ );
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionServerPolicy::registerExecuteCB( __Args&&... args )
{
    _ExecuteCallbackPolicy::registerCallback( std::forward<__Args>( args )... );
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionServerPolicy::registerPreemptCB( _PreemptCallback const & callback )
{
    preempt_callback_ = callback;
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionServerPolicy::registerGoalCB( _GoalCallback const & callback )
{
    goal_callback_ = callback;
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
void __ActionServerPolicy::updateResult( __ActionServerPolicy::_ResultMsg && result, std::string && result_info )
{
    result_msg_ = std::forward<__ActionServerPolicy::_ResultMsg>( result );
    result_info_ = std::forward<std::string>( result_info );
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionServerPolicy::setSuccessful( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    PRINT_INFO( "Set goal [%s] on topic [%s] to successful", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );

    result_type_ = _GoalStatusMsg::SUCCEEDED;

    updateResult( std::forward<__Args>( args )... );
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionServerPolicy::setAborted( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    PRINT_INFO( "Set goal [%s] on topic [%s] to aborted", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );

    result_type_ = _GoalStatusMsg::ABORTED;

    updateResult( std::forward<__Args>( args )... );
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionServerPolicy::setPreempted( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    PRINT_INFO( "Set goal [%s] on topic [%s] to preempted", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );

    result_type_ = _GoalStatusMsg::PREEMPTED;

    updateResult( std::forward<__Args>( args )... );

    preempt_accepted_ = true;
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionServerPolicy::completeAction( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    setSuccessful( std::forward<__Args>( args )... );

    action_mutex_.unlock();
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionServerPolicy::abortAction( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    setAborted( std::forward<__Args>( args )... );

    action_mutex_.unlock();
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionServerPolicy::preemptAction( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    setPreempted( std::forward<__Args>( args )... );

    action_mutex_.unlock();
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
bool __ActionServerPolicy::waitOnAction( double const & wait_time )
{
    QUICKDEV_ASSERT_INITIALIZED( false );

    if( wait_time > 0 )
    {
        auto action_timed_lock = quickdev::make_unique_lock( action_mutex_, quickdev::Duration( wait_time ) );
    }
    else action_mutex_.lock();

    return !preemptAccepted();
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
bool __ActionServerPolicy::preemptRequested()
{
    QUICKDEV_ASSERT_INITIALIZED();

    return action_server_->isPreemptRequested();
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
bool const & __ActionServerPolicy::preemptAccepted() const
{
    return preempt_accepted_;
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
bool __ActionServerPolicy::successful() const
{
    return !preemptAccepted();
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
bool __ActionServerPolicy::active()
{
    QUICKDEV_ASSERT_INITIALIZED( false );

    return action_server_->isActive();
}

// =========================================================================================================================================
template<class __Action, unsigned int __Id__>
bool __ActionServerPolicy::initialized()
{
    QUICKDEV_ASSERT_INITIALIZED( false );

    return goal_initialized_;
}

#undef __ActionServerPolicy
