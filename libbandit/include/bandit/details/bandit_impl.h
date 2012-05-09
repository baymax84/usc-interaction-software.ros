// =============================================================================================================================================
template<class __Exception, class... __Args>
static void BANDIT_EXCEPT( std::string && msg, __Args&&... args )
{
    char buf[256];
    snprintf( buf, 256, ( std::string( "bandit::Bandit::%s: " ) + msg ).c_str(), __FUNCTION__, args... );
    throw __Exception( buf );
}

// =============================================================================================================================================
template<class... __Args>
void Bandit::addJoint( __Args&&... args )
{
    Joint joint( args... );

    joints_map_[joint.name] = joint;

    master_.setJointType( joint.mod_id, joint.type );
}

// =============================================================================================================================================
template<class... __Args>
void Bandit::setJointPIDConfig( JointName && name, __Args&&... args )
{
    auto joint_it = getJoint( name );

    if( joint_it->second.type != smartservo::SMART_SERVO ) BANDIT_EXCEPT<BanditException>( "Tried to set PID gains on: %s, which is not a SMART_SERVO\n", name.name_.c_str() );

    master_.setJointPIDConfig( joint_it->second.mod_id, joint_it->second.which, args... );
}
