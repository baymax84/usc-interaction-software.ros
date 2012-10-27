#define __ActionServerPolicy ActionServerPolicy<__Action, __Id__, __Storage>
#define __template_ACTION_SERVER_POLICY template<class __Action, unsigned int __Id__, class __Storage>

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
QUICKDEV_DECLARE_INIT( ActionServerPolicy<__Action, __Id__, __Storage>:: )
{
    auto & nh_rel = NodeHandlePolicy::getNodeHandle();

    auto const enable_key_ids( getMetaParamDef<bool>( "enable_key_ids", false, std::forward<__Args>( args )... ) );

    auto const action_name = policy::readPolicyParamAuto<std::string>( nh_rel, enable_key_ids, "action_name_param", __Id__, "action_name", "action", args... );
    auto const use_execute_callback = policy::readPolicyParamAuto<bool>( nh_rel, enable_key_ids, "use_execute_callback_param", __Id__, "use_execute_callback", true, args... );

    ros::NodeHandle action_nh( nh_rel, action_name );

    action_topic_name_ = ros::NodeHandle( nh_rel, action_name ).getNamespace();

    PRINT_INFO( "Creating action server [%s] on topic [%s]", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );

    if( use_execute_callback ) action_server_ptr_ = boost::make_shared<_ActionServer>( nh_rel, action_name, auto_bind( &ActionServerPolicy::executeActionCB, this ), false );
    else
    {
        action_server_ptr_ = boost::make_shared<_ActionServer>( nh_rel, action_name, false );
        action_server_ptr_->registerGoalCallback( auto_bind( &ActionServerPolicy::goalCB, this ) );
    }

    action_server_ptr_->registerPreemptCallback( auto_bind( &ActionServerPolicy::preemptCB, this ) );

    action_server_ptr_->start();

    QUICKDEV_SET_INITIALIZED();
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
QUICKDEV_DECLARE_MESSAGE_CALLBACK( (ActionServerPolicy<__Action, __Id__, __Storage>::executeActionCB), typename _GoalMsg )
{
    QUICKDEV_ASSERT_INITIALIZED();

    PRINT_INFO( "Got goal callback [%s] on topic [%s]", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );

    if( !action_server_ptr_ )
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

        execute_active_ = true;

        // unblock new goal mutex if it's waiting
        wait_for_goal_mutex_.unlock();

        // work gets done here; the current context is guaranteed to be a separate thread, so it's safe to block here
        _ExecuteCallbackPolicy::invokeCallback( msg, std::forward<decltype( action_server_ptr_ )>( action_server_ptr_ ) );

        execute_active_ = false;

        switch( result_type_ )
        {
        case _GoalStatusMsg::SUCCEEDED:
            action_server_ptr_->setSucceeded( result_msg_, result_info_ );
            break;
        case _GoalStatusMsg::ABORTED:
            action_server_ptr_->setAborted( result_msg_, result_info_ );
            break;
        case _GoalStatusMsg::PREEMPTED:
            action_server_ptr_->setPreempted( result_msg_, result_info_ );
            break;
        }
    }
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
void __ActionServerPolicy::preemptCB()
{
    if( preempt_callback_ ) preempt_callback_( action_server_ptr_ );
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
void __ActionServerPolicy::goalCB()
{
    if( goal_callback_ ) goal_callback_( action_server_ptr_ );
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
template<class... __Args>
void __ActionServerPolicy::registerExecuteCB( __Args&&... args )
{
    _ExecuteCallbackPolicy::registerCallback( std::forward<__Args>( args )... );
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
void __ActionServerPolicy::registerPreemptCB( _PreemptCallback const & callback )
{
    preempt_callback_ = callback;
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
void __ActionServerPolicy::registerGoalCB( _GoalCallback const & callback )
{
    goal_callback_ = callback;
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
template<class... __Args>
void __ActionServerPolicy::sendFeedback( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    action_server_ptr_->publishFeedback( std::forward<__Args>( args )... );
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
void __ActionServerPolicy::updateResult( __ActionServerPolicy::_ResultMsg const & result, std::string const & result_info )
{
    result_msg_ = result;
    result_info_ = result_info;
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
template<class... __Args>
void __ActionServerPolicy::setSuccessful( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    PRINT_INFO( "Set goal [%s] on topic [%s] to successful", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );

    result_type_ = _GoalStatusMsg::SUCCEEDED;

    updateResult( std::forward<__Args>( args )... );

    execute_active_ = false;
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
template<class... __Args>
void __ActionServerPolicy::setAborted( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    PRINT_INFO( "Set goal [%s] on topic [%s] to aborted", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );

    result_type_ = _GoalStatusMsg::ABORTED;

    updateResult( std::forward<__Args>( args )... );

    execute_active_ = false;
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
template<class... __Args>
void __ActionServerPolicy::setPreempted( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    PRINT_INFO( "Set goal [%s] on topic [%s] to preempted", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );

    result_type_ = _GoalStatusMsg::PREEMPTED;

    updateResult( std::forward<__Args>( args )... );

    preempt_accepted_ = true;
    execute_active_ = false;
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
template<class... __Args>
void __ActionServerPolicy::completeAction( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    setSuccessful( std::forward<__Args>( args )... );

    action_mutex_.unlock();
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
template<class... __Args>
void __ActionServerPolicy::abortAction( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    setAborted( std::forward<__Args>( args )... );

    action_mutex_.unlock();
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
template<class... __Args>
void __ActionServerPolicy::preemptAction( __Args&&... args )
{
    QUICKDEV_ASSERT_INITIALIZED();

    setPreempted( std::forward<__Args>( args )... );

    action_mutex_.unlock();
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
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
__template_ACTION_SERVER_POLICY
void __ActionServerPolicy::waitForNewGoal( double const & wait_time )
{
    QUICKDEV_ASSERT_INITIALIZED();

    // make sure the mutex is locked once
    wait_for_goal_mutex_.try_lock();

    if( wait_time > 0 )
    {
        // block and wait for external unblock; time out after @wait_time
        auto wait_for_goal_timed_lock = quickdev::make_unique_lock( wait_for_goal_mutex_, quickdev::Duration( wait_time ) );
    }
    // block and wait for external unblock
    else wait_for_goal_mutex_.lock();

    // unlock
    wait_for_goal_mutex_.unlock();
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
bool __ActionServerPolicy::preemptRequested()
{
    QUICKDEV_ASSERT_INITIALIZED();

    return action_server_ptr_->isPreemptRequested();
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
bool const & __ActionServerPolicy::preemptAccepted() const
{
    return preempt_accepted_;
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
bool __ActionServerPolicy::successful() const
{
    return !preemptAccepted();
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
bool __ActionServerPolicy::active()
{
    QUICKDEV_ASSERT_INITIALIZED( false );

    // action_server_ptr_->isActive();
    return execute_active_;
}

// =========================================================================================================================================
__template_ACTION_SERVER_POLICY
bool __ActionServerPolicy::initialized()
{
    QUICKDEV_ASSERT_INITIALIZED( false );

    return goal_initialized_;
}

#undef __ActionServerPolicy
#undef __template_ACTION_SERVER_POLICY
