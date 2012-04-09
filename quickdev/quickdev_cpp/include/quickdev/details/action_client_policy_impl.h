#define __ActionClientPolicy ActionClientPolicy<__Action, __Id__>

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
__ActionClientPolicy::~ActionClientPolicy()
{
    if( action_client_ ) delete action_client_;
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
QUICKDEV_DECLARE_INIT( ActionClientPolicy<__Action, __Id__>:: )
{
    auto nh_rel = NodeHandlePolicy::getNodeHandle();

    auto const enable_key_ids( getMetaParamDef<bool>( "enable_key_ids", false, std::forward<__Args>( args )... ) );

    action_name_ = policy::readPolicyParamAuto<std::string>( nh_rel, enable_key_ids, "action_name_param", __Id__, "action_name", "action", std::forward<__Args>( args )... );

    action_topic_name_ = ros::NodeHandle( nh_rel, action_name_ ).getNamespace();

    PRINT_INFO( "Creating action client [%s] on topic [%s]", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );

    action_client_ = new _ActionClient( nh_rel, action_name_ );

    QUICKDEV_SET_INITIALIZED();
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionClientPolicy::activeCB()
{
    PRINT_INFO( "Activated goal [%s] on topic [%s]", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );

    _ActiveCallbackPolicy::invokeCallback();
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
QUICKDEV_DECLARE_MESSAGE_CALLBACK2( (ActionClientPolicy<__Action, __Id__>::feedbackCB), typename _FeedbackMsg, feedback )
{
    _FeedbackCallbackPolicy::invokeCallback( feedback );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionClientPolicy::doneCB( _GoalState const & state, typename _ResultMsg::ConstPtr const & result )
{
    PRINT_INFO( "Finished [%s] on topic [%s] in state [%s]", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str(), state.toString().c_str() );

    _DoneCallbackPolicy::invokeCallback( state, result );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionClientPolicy::registerActiveCB( __Args&&... args )
{
    _ActiveCallbackPolicy::registerCallback( std::forward<__Args>( args )... );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionClientPolicy::registerFeedbackCB( __Args&&... args )
{
    _FeedbackCallbackPolicy::registerCallback( std::forward<__Args>( args )... );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
template<class... __Args>
void __ActionClientPolicy::registerDoneCB( __Args&&... args )
{
    _DoneCallbackPolicy::registerCallback( std::forward<__Args>( args )... );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionClientPolicy::registerTimeout( double const & duration, _TimeoutCallback const & callback )
{
    registerTimeout( ros::Duration( duration ), callback );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionClientPolicy::registerTimeout( ros::Duration const & duration, _TimeoutCallback const & callback )
{
    QUICKDEV_ASSERT_INITIALIZED();

    if( duration.isZero() )
    {
        PRINT_WARN( "Registering a timeout with a duration of zero has no effect" );
        return;
    }

    timeout_timestamp_ = ros::Time::now() + duration;
    timeout_callback_ = callback;
    enable_timeout_ = true;
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
bool __ActionClientPolicy::sendGoalAndWait( _GoalMsg const & goal, double const & duration )
{
    return sendGoalAndWait( goal, ros::Duration( duration ) );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
bool __ActionClientPolicy::sendGoalAndWait( _GoalMsg const & goal, ros::Duration const & duration )
{
    sendGoal( goal );
    return waitForResult( duration );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionClientPolicy::sendGoalAndWait( _GoalMsg const & goal, double const & duration, _TimeoutCallback const & callback )
{
    sendGoalAndWait( goal, ros::Duration( duration ), callback );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionClientPolicy::sendGoalAndWait( _GoalMsg const & goal, ros::Duration const & duration, _TimeoutCallback const & callback )
{
    sendGoal( goal );
    waitForResult( duration, callback );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
bool __ActionClientPolicy::waitForResult( double const & duration )
{
    return waitForResult( ros::Duration( duration ) );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
bool __ActionClientPolicy::waitForResult( ros::Duration const & duration )
{
    QUICKDEV_ASSERT_INITIALIZED( false );

    return action_client_->waitForResult( duration );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionClientPolicy::waitForResult( double const & duration, _TimeoutCallback const & callback )
{
    return waitForResult( ros::Duration( duration ), callback );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionClientPolicy::waitForResult( ros::Duration const & duration, _TimeoutCallback const & callback )
{
    registerTimeout( duration, callback );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
typename __ActionClientPolicy::_GoalState
__ActionClientPolicy::getState()
{
    QUICKDEV_CHECK_INITIALIZED();

    return action_client_->getState();
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
typename __ActionClientPolicy::_ResultMsg::ConstPtr
__ActionClientPolicy::getResult()
{
    QUICKDEV_CHECK_INITIALIZED();

    return action_client_->getResult();
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionClientPolicy::sendGoal( _GoalMsg const & goal )
{
    QUICKDEV_ASSERT_INITIALIZED();

    if( !action_client_ )
    {
        PRINT_ERROR( "Cannot send goal to un-initialized client" );
        return;
    }

    PRINT_INFO( "Waiting for action server [%s] on topic [%s] to start...", QUICKDEV_GET_MESSAGE_NAME( __Action ).c_str(), action_topic_name_.c_str() );
    action_client_->waitForServer();

    PRINT_INFO( "Action server started; sending goal..." );
    // send a goal to the action
    action_client_->sendGoal( goal, auto_bind( &ActionClientPolicy::doneCB, this ), auto_bind( &ActionClientPolicy::activeCB, this ), auto_bind( &ActionClientPolicy::feedbackCB, this ) );
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionClientPolicy::update()
{
    QUICKDEV_ASSERT_INITIALIZED();

    if( enable_timeout_ && !timeout_timestamp_.isZero() && ros::Time::now() > timeout_timestamp_ )
    {
        enable_timeout_ = false;
        timeout_callback_();
    }
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
void __ActionClientPolicy::interruptAction()
{
    QUICKDEV_ASSERT_INITIALIZED();

    action_client_->cancelGoal();
}

// =============================================================================================================================================
template<class __Action, unsigned int __Id__>
bool __ActionClientPolicy::successful()
{
    QUICKDEV_CHECK_INITIALIZED();

    return action_client_->getState() == _GoalState::SUCCEEDED;
}

#undef __ActionClientPolicy
