/***************************************************************************
 *  include/quickdev/macros.h
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

#ifndef QUICKDEVCPP_QUICKDEV_MACROS_H_
#define QUICKDEVCPP_QUICKDEV_MACROS_H_


// ########## Generic Policy Macros ####################################
// ---------------------------------------------------------------------
#define QUICKDEV_MAKE_POLICY_NAME( PolicyNameBase ) \
/*! \brief Auto-generated function to return the name of this policy */ \
/*! \return \code "PolicyNameBase" \endcode */ \
public: const static inline std::string name() { return #PolicyNameBase; }

// ---------------------------------------------------------------------
// somehow this actually works for templated classes... [somuchwin]
// So if you had: template<class SomeType> class SomePolicy{};
// And: class SomeOtherPolicy : public SomePolicy{};
// Within SomeOtherPolicy, you can say: auto & instance = SomePolicy::getInstance();
#define QUICKDEV_MAKE_POLICY_REFERENCE( PolicyNameBase ) \
/*! \brief Auto-generated function to return a reference to this policy */ \
/*! \details Used by child policies to get a reference to their parent PolicyNameBase##Policy */ \
/*! \return a reference to PolicyNameBase##Policy */ \
public: inline PolicyNameBase##Policy & getInstance() { return *this; }

// ---------------------------------------------------------------------
#define QUICKDEV_MAKE_POLICY_FUNCS( PolicyNameBase ) \
QUICKDEV_MAKE_POLICY_NAME( PolicyNameBase ) \
QUICKDEV_MAKE_POLICY_REFERENCE( PolicyNameBase )

// ---------------------------------------------------------------------
#define QUICKDEV_GET_POLICY_NAMESPACE( PolicyNameBase ) \
PolicyNameBase##Policy_types
// ---------------------------------------------------------------------
#define QUICKDEV_GET_POLICY_NS( PolicyNameBase ) \
QUICKDEV_GET_POLICY_NAMESPACE( PolicyNameBase )

// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_POLICY_NAMESPACE( PolicyNameBase ) \
/*! \brief The "private" namespace for PolicyNameBase##Policy */ \
namespace QUICKDEV_GET_POLICY_NAMESPACE( PolicyNameBase )
// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_POLICY_NS( PolicyNameBase ) \
QUICKDEV_DECLARE_POLICY_NAMESPACE( PolicyNameBase )

// ---------------------------------------------------------------------
#define QUICKDEV_GET_POLICY_ADAPTER( PolicyNameBase ) \
PolicyNameBase##PolicyAdapter

// ########## Pipeline for Policy with no dependent types ##############
// ---------------------------------------------------------------------
#define QUICKDEV_GET_POLICY_ADAPTER_WITH_NS( PolicyNameBase ) \
QUICKDEV_GET_POLICY_NS( PolicyNameBase )::QUICKDEV_GET_POLICY_ADAPTER( PolicyNameBase )

// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_POLICY( PolicyNameBase, __Policies... ) \
QUICKDEV_DECLARE_POLICY_NS( PolicyNameBase ) \
{ \
    /*! \brief The type of GenericPolicyAdapter used by PolicyNameBase##Policy */ \
    /*! \details Specifically, PolicyNameBase##Policy utilizes: __Policies */ \
    typedef quickdev::GenericPolicyAdapter< __Policies > QUICKDEV_GET_POLICY_ADAPTER( PolicyNameBase ); \
}

// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_POLICY_CLASS( PolicyNameBase ) \
/*! \brief Class declaration for PolicyNameBase##Policy */ \
/*! \details Specifically, we utilize the "private" policy namespace QUICKDEV_GET_POLICY_NS( PolicyNameBase ) and inherit from the policy adapter it references: QUICKDEV_GET_POLICY_ADAPTER_WITH_NS( PolicyNameBase ) */ \
class PolicyNameBase##Policy : public QUICKDEV_GET_POLICY_ADAPTER_WITH_NS( PolicyNameBase )

// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( PolicyNameBase ) \
/*! \brief Constructor declaration for PolicyNameBase##Policy */ \
/*! \details Specifically, we utilize the "private" policy namespace QUICKDEV_GET_POLICY_NS( PolicyNameBase ) and call the constructor for the policy adapter it references: QUICKDEV_GET_POLICY_ADAPTER_WITH_NS( PolicyNameBase ) */ \
/*! \tparam __Args the variadic template of argument types to pass on to the parent policies. */ \
/*! \param args the variadic template of arguments to pass on to the parent policies. */ \
/*! \note in reality, the only argument passed through the constructor via __Args is a ros::NodeHandle due to unsupported features in GCC \see GenericPolicyAdapter() */ \
/*! \note the given ros::NodeHandle can be extracted via getFirstOfType(): \code auto nh_rel = getFirstOfType<ros::NodeHandle>( args... ) \endcode */ \
public: \
    template<class... __Args> \
    PolicyNameBase##Policy( __Args&&... args ) \
    : \
        QUICKDEV_GET_POLICY_ADAPTER_WITH_NS( PolicyNameBase )( args... )

// ########## Pipeline for Policy with dependent types #################
// ---------------------------------------------------------------------
#define QUICKDEV_GET_POLICY_ADAPTER2( PolicyNameBase, __Types... ) \
QUICKDEV_GET_POLICY_ADAPTER( PolicyNameBase )<__Types>

// ---------------------------------------------------------------------
#define QUICKDEV_GET_POLICY_ADAPTER_WITH_NS2( PolicyNameBase, __Types... ) \
QUICKDEV_GET_POLICY_ADAPTER2( PolicyNameBase, __Types )

// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_POLICY2( PolicyNameBase, __Policies... ) \
/*! \brief The "private" namespace for PolicyNameBase##Policy */ \
/*! \details A special alternative means of declaring a "private namespace"  used when a typed policy has parent policies which depend on one or more of its types. Specifically, we create this struct instead of creating \code QUICKDEV_GET_POLICY_NS( PolicyNameBase ) \endcode */ \
struct QUICKDEV_GET_POLICY_ADAPTER( PolicyNameBase ) \
{ \
    /*! \brief The type of GenericPolicyAdapter used by PolicyNameBase##Policy */ \
    /*! \details Specifically, PolicyNameBase##Policy utilizes: __Policies */ \
    typedef quickdev::GenericPolicyAdapter< __Policies > type; \
};

// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_POLICY_CLASS2( PolicyNameBase, __Types... ) \
/*! \brief Class declaration for PolicyNameBase##Policy */ \
/*! \details Specifically, we utilize the "private namespace" QUICKDEV_GET_POLICY_ADAPTER2( PolicyNameBase, __Types ) and inherit from the policy adapter it references: QUICKDEV_GET_POLICY_ADAPTER_WITH_NS2( PolicyNameBase, __Types )::type */ \
class PolicyNameBase##Policy : public QUICKDEV_GET_POLICY_ADAPTER_WITH_NS2( PolicyNameBase, __Types )::type

// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_POLICY_CONSTRUCTOR2( PolicyNameBase, __Types... ) \
/*! \brief Constructor declaration for PolicyNameBase##Policy */ \
/*! \details Specifically, we utilize the "private namespace" QUICKDEV_GET_POLICY_ADAPTER2( PolicyNameBase, __Types ) and call the constructor for the policy adapter it references: QUICKDEV_GET_POLICY_ADAPTER_WITH_NS2( PolicyNameBase, __Types )::type */ \
/*! \tparam __Args the variadic template of argument types to pass on to the parent policies. */ \
/*! \param args the variadic template of arguments to pass on to the parent policies. */ \
/*! \note in reality, the only argument passed through the constructor via __Args is a ros::NodeHandle due to unsupported features in GCC \see GenericPolicyAdapter() */ \
/*! \note the given ros::NodeHandle can be extracted via getFirstOfType(): \code auto nh_rel = getFirstOfType<ros::NodeHandle>( args... ) \endcode */ \
public: \
    template<class... __Args> \
    PolicyNameBase##Policy( __Args&&... args ) \
    : \
        QUICKDEV_GET_POLICY_ADAPTER_WITH_NS2( PolicyNameBase, __Types )::type( args... )

// ########## Generic Node Macros ######################################
// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_NODE( NodeNameBase, __Policies... ) \
/*! \brief The type of Node used by NodeNameBase##Node */ \
/*! \details Specifically, NodeNameBase##Node utilizes: __Policies */ \
typedef quickdev::Node< __Policies > _##NodeNameBase##NodeAdapterType;

// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_NODE_CLASS( NodeNameBase ) \
/*! \brief Class declaration for NodeNameBase##Node */ \
/*! \details Specifically, we utilize the type _##NodeNameBase##NodeAdapterType, declared earlier and  inherit the node adapter it aliases. */ \
class NodeNameBase##Node : public _##NodeNameBase##NodeAdapterType

// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_NODE_CONSTRUCTOR( NodeNameBase ) \
/*! \brief Constructor declaration for NodeNameBase##Node */ \
/*! \details Specifically, we utilize the type _##NodeNameBase##NodeAdapterType, declared earlier and call the constructor for the node adapter it aliases. */ \
/*! \tparam __Args the variadic template of argument types to pass on to the parent policies. */ \
/*! \param args the variadic template of arguments to pass on to the parent policies. */ \
/*! \note in reality, the only argument passed through the constructor via __Args is a ros::NodeHandle due to unsupported features in GCC \see GenericPolicyAdapter() */ \
/*! \note the given ros::NodeHandle can be extracted via getFirstOfType(): \code auto nh_rel = getFirstOfType<ros::NodeHandle>( args... ) \endcode */ \
public: \
    template<class... __Args> \
    NodeNameBase##Node( __Args&&... args ) \
    : \
        _##NodeNameBase##NodeAdapterType( args... )

// ########## Node Instantiation Macros ################################
// ---------------------------------------------------------------------
// use: QUICKDEV_INST_NODE( SomeNode, "some_node" )
#define QUICKDEV_INST_NODE( NodeClassname, node_name_string ) \
/*! \brief Instantiate NodeClassname */ \
/*! \details Initialize ROS (calling this node node_name_string), make a new relative (or "private") nodehandle, then create a new instance of NodeClassname and start it up. */ \
int main( int argc, char ** argv ) \
{ \
    ros::init( argc, argv, node_name_string ); \
    ros::NodeHandle nh( "~" ); \
    \
    NodeClassname node_inst( nh ); \
    node_inst.spin(); \
    return 0; \
}

// ########## Generic Nodelet Macros ###################################
// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_NODELET( namespace_name, ClassName ) \
/*! \brief The nodelet namespace for the package namespace_name */ \
namespace namespace_name { \
/*! \brief Class declaration for ClassName##Nodelet */ \
/*! \details Specifically, ClassName##Nodelet is simply a Nodelet wrapper around ClassName##Node */ \
class ClassName##Nodelet : public quickdev::Nodelet<ClassName##Node>{}; }

// ########## Nodelet Instantiation Macros #############################
// ---------------------------------------------------------------------
#define QUICKDEV_INST_NODELET( namespace_name, ClassName, nodelet_name ) \
/*! \brief "Instantiate" namespace_name::ClassName */ \
/*! \details Register namespace_name::ClassName with pluginlib as nodelet_name */ \
PLUGINLIB_DECLARE_CLASS( namespace_name, nodelet_name, namespace_name::ClassName##Nodelet, nodelet::Nodelet )

// ########## Initable Policy Macros ###################################
// ---------------------------------------------------------------------
#define QUICKDEV_ENABLE_INIT() \
/*! \brief Used to determine whether a policy is initializable */ \
public: const static bool HAS_INIT_ = true; \
/*! \brief Used to determine whether a policy has been initialized */ \
/*! \note This variable needs to be manually set to "false" at construction */ \
private: bool initialized_; \
/*! \brief Used to set the initialization state of an initializable policy */ \
/*! \details Usually called as the last line in init() */ \
/*! \param value the new initialization state of this policy */ \
private: inline void setInitialized( const bool & value ){ initialized_ = value; } \
/*! \brief Used to pass any post-construction values, usually meta-params, to this policy */ \
/*! \details Meta-params passed through this function can be extracted with getFirstOfType(), getMetaParam(), or getMetaParamDef() */ \
/*! \tparam __Args the variadic template of types associated with args */ \
/*! \param args the variadic template of values given to this policy to parse, if applicable */ \
public: template<class... __Args> \
void init( __Args&&... args )

// ---------------------------------------------------------------------
#define QUICKDEV_ASSERT_INITIALIZED( return_val ) \
QUICKDEV_CHECK_INITIALIZED(); \
if( !initialized_ ) return return_val
// ---------------------------------------------------------------------
#define QUICKDEV_CHECK_INITIALIZED() \
if( !initialized_ ) PRINT_ERROR( "Policy [%s] has not been initialized!", name().c_str() ); \
if( !initialized_ ) PRINT_ERROR( "Some functionality may be disabled." )

// ---------------------------------------------------------------------
#define QUICKDEV_SET_INITIALIZED() \
this->setInitialized( true )

// ########## Updateable Policy Macros #################################
// ---------------------------------------------------------------------
#define QUICKDEV_ENABLE_UPDATE \
const static bool HAS_UPDATE_ = true; \
template<class... __Args> \
void update( __Args&&... args )

// ########## Generic Callback Macros ##################################
// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_MESSAGE_CALLBACK2( callbackName, __MessageType, message_name ) \
/*! \brief Callback for a __MessageType message */ \
/*! \param message_name a pointer to the incoming message */ \
/*! \return nothing */ \
void callbackName( const __MessageType::ConstPtr & message_name )
// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_MESSAGE_CALLBACK( callbackName, __MessageType ) \
QUICKDEV_DECLARE_MESSAGE_CALLBACK2( callbackName, __MessageType, msg )

// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_CONDITIONAL_MESSAGE_CALLBACK2( callbackName, __MessageType, message_name, condition ) \
/*! \brief Callback for a __MessageType message, enabled if condition */ \
/*! \param message_name a pointer to the incoming message */ \
/*! \return nothing */ \
typename std::enable_if<condition, void>::type \
callbackName( const __MessageType::ConstPtr & message_name )
// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_CONDITIONAL_MESSAGE_CALLBACK( callbackName, __MessageType, condition ) \
QUICKDEV_DECLARE_CONDITIONAL_MESSAGE_CALLBACK2( callbackName, __MessageType, msg, condition )

// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_SERVICE_CALLBACK2( callbackName, __ServiceType, request_name, response_name ) \
/*! \brief Callback for a __ServiceType service */ \
/*! \param request_name a pointer to the incoming request */ \
/*! \param response_name a pointer to the outgoing response */ \
/*! \return true if the request could be completed and false otherwise */ \
bool callbackName( __ServiceType::Request & request_name, __ServiceType::Response & response_name )
// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_SERVICE_CALLBACK( callbackName, __ServiceType ) \
QUICKDEV_DECLARE_SERVICE_CALLBACK2( callbackName, __ServiceType, request, response )

// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_RECONFIGURE_CALLBACK2( callbackName, __ReconfigureType, config_name, level_name ) \
/*! \brief Callback for a __ReconfigureType dynamic reconfigure parameter message */ \
/*! \param config_name a reference to the incoming reconfigure parameters */ \
/*! \param level_name the incoming level */ \
/*! \return nothing */ \
void callbackName( __ReconfigureType & config_name, uint32_t level_name )
// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_RECONFIGURE_CALLBACK( callbackName, __ReconfigureType ) \
QUICKDEV_DECLARE_RECONFIGURE_CALLBACK2( callbackName, __ReconfigureType, config, level )

// ########## ImageProc Policy Macros ##################################
// ---------------------------------------------------------------------
#define IMAGE_PROC_PROCESS_IMAGE( image_ptr_name ) \
/*! \brief Callback for an image processing node */ \
/*! \details All the actual processing work is triggered from within this callback */ \
void processImage( cv_bridge::CvImageConstPtr & image_ptr_name )

// ########## Runable Policy Macros ####################################
// ---------------------------------------------------------------------
#define QUICKDEV_SPIN_FIRST() \
/*! \brief Function usually used for node / policy initialization */ \
/*! \details Called a single time post-construction but prior to the main loop by RunablePolicy::spin() */ \
/*! \sa RunablePolicy */ \
void spinFirst()

// ---------------------------------------------------------------------
#define QUICKDEV_SPIN_ONCE() \
/*! \brief Function usually used as a node's main loop */ \
/*! \details Called every cycle at an (ideally) fixed frequency by RunablePolicy */ \
/*! \sa RunablePolicy */ \
void spinOnce()

// ########## Type Enable/Disable Macros ###############################
// ---------------------------------------------------------------------

#define QUICKDEV_ENABLE_IF( __ReturnType, condition ) \
typename std::enable_if<(condition), __ReturnType>::type

// ---------------------------------------------------------------------
#define QUICKDEV_ENABLE_IF_SAME( __ReturnType, __Type1, __Type2 ) \
QUICKDEV_ENABLE_IF( __ReturnType, ( std::is_same<__Type1, __Type2>::value ) )
// "disable if not same" is an alias for "enable if same"
#define QUICKDEV_DISABLE_IF_NOT_SAME QUICKDEV_ENABLE_IF_SAME

// ---------------------------------------------------------------------
#define QUICKDEV_DISABLE_IF_SAME( __ReturnType, __Type1, __Type2 ) \
QUICKDEV_ENABLE_IF( __ReturnType, ( !std::is_same<__Type1, __Type2>::value ) )
// "enable_if_not_same" is an alias for "disable_if_same"
#define QUICKDEV_ENABLE_IF_NOT_SAME QUICKDEV_DISABLE_IF_SAME

// use pair: ENABLE_IF_SAME  / ENABLE_IF_NOT_SAME
// or:       DISABLE_IF_SAME / DISABLE_IF_NOT_SAME

// ########## Threading Macros #########################################
// ---------------------------------------------------------------------
#define QUICKDEV_TRY_LOCK_OR_WARN2( lock_name, args... ) \
if( !lock_name ) PRINT_WARN( "Lock is busy. " args )

#define QUICKDEV_TRY_LOCK_OR_WARN( args... ) \
QUICKDEV_TRY_LOCK_OR_WARN2( lock, args )

// ---------------------------------------------------------------------
#define QUICKDEV_TRY_LOCK_OR_RETURN2( lock_name, args... ) \
QUICKDEV_TRY_LOCK_OR_WARN2( lock_name, args ); \
if( !lock_name ) return

#define QUICKDEV_TRY_LOCK_OR_RETURN( args... ) \
QUICKDEV_TRY_LOCK_OR_RETURN2( lock, args )

// ---------------------------------------------------------------------
#define QUICKDEV_LOCK_MUTEX2( lock_name, mutex ) \
auto lock_name = mutex.lock()

#define QUICKDEV_LOCK_MUTEX( mutex ) \
QUICKDEV_LOCK_MUTEX2( lock, mutex )

// ---------------------------------------------------------------------
#define QUICKDEV_LOCK_CACHE_AND_GET( cache, output ) \
QUICKDEV_LOCK_MUTEX( cache ); \
const auto & output = cache.get()

// ---------------------------------------------------------------------
#define QUICKDEV_TRY_UPDATE_CACHE2( lock_name, cache, value ) \
auto lock_name = cache.tryLockAndUpdate( value )

// ---------------------------------------------------------------------
#define QUICKDEV_TRY_UPDATE_CACHE( cache, value ) \
QUICKDEV_TRY_UPDATE_CACHE2( lock, cache, value )


// ########## Internal Macros ##########################################
// ---------------------------------------------------------------------
#define QUICKDEV_GET_INTERNAL_NAMESPACE() \
quickdev
// ---------------------------------------------------------------------
#define QUICKDEV_DECLARE_INTERNAL_NAMESPACE() \
/*! \brief internal library namespace for quickdev */ \
namespace QUICKDEV_GET_INTERNAL_NAMESPACE()

// ---------------------------------------------------------------------
#define __QUICKDEV_FUNCTION_TYPE \
std::function

// ########## ROS Message Macros #######################################
// ---------------------------------------------------------------------
#define QUICKDEV_GET_MESSAGE_NAME( __Message ) \
std::string( ros::message_traits::DataType<__Message>::value() )

// ---------------------------------------------------------------------
#define QUICKDEV_GET_MESSAGE_INST_NAME( msg ) \
QUICKDEV_GET_MESSAGE_NAME( decltype( quickdev::getMessageType( msg ) ) )

// ########## ROS Message Macros #######################################
// ---------------------------------------------------------------------
#define QUICKDEV_GET_SERVICE_NAME( __Service ) \
std::string( ros::service_traits::DataType<__Service>::value() )

// ########## General Utility Macros ###################################
// ---------------------------------------------------------------------
#define QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_name ) \
auto & nh_name = QUICKDEV_GET_INTERNAL_NAMESPACE()::RunablePolicy::getNodeHandle()
// ---------------------------------------------------------------------
#define QUICKDEV_GET_NODEHANDLE( nh_name ) \
auto & nh_name = QUICKDEV_GET_INTERNAL_NAMESPACE()::NodeHandlePolicy::getNodeHandle()

#endif // QUICKDEVCPP_QUICKDEV_MACROS_H_
