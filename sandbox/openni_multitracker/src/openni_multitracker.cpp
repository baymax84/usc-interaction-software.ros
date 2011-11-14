#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include <openni_multitracker/UserStateArray.h>

#include <map>
#include <sstream>

using std::string;

xn::Context
	context_;
	
xn::DepthGenerator
	depth_generator_;
	
xn::UserGenerator
	user_generator_;

// flag specifying whether or not calibration is required for new users
// this flag is set by 'user_generator_.GetSkeletonCap().NeedPoseForCalibration()'
XnBool
	need_pose_for_calibration_ = false;
// name of the required calibraiton pose, if any
// this flag is set by 'user_generator_.GetSkeletonCap().GetCalibrationPose( calibration_pose_ )'
XnChar
	calibration_pose_[20] = "";

struct UserState
{
	bool is_tracked_;
	
	UserState() : is_tracked_( false )
	{
		//
	}
};

typedef
	std::map<
		unsigned int,
		UserState>
	_UserStateMap;

_UserStateMap user_states_;

ros::Publisher user_states_pub_;

void XN_CALLBACK_TYPE
	newUserCB(
		xn::UserGenerator& generator,
		XnUserID id,
		void* cookie )
{
	printf(
		"New User %d\n",
		id );

	if ( need_pose_for_calibration_ )
		user_generator_.GetPoseDetectionCap().StartPoseDetection(
			calibration_pose_,
			id );
	else
		user_generator_.GetSkeletonCap().RequestCalibration(
			id,
			TRUE );
}

void
	XN_CALLBACK_TYPE lostUserCB(
		xn::UserGenerator& generator,
		XnUserID id,
		void* cookie )
{
	printf(
		"Lost user %d\n",
		id );
		
	user_states_[id].is_tracked_ = false;
}

void XN_CALLBACK_TYPE
	startCalibrationCB(
		xn::SkeletonCapability& capability,
		XnUserID id,
		void* cookie )
{
	printf(
		"Calibration started for user %d\n",
		id );
}

void XN_CALLBACK_TYPE
	endCalibrationCB( 
		xn::SkeletonCapability& capability,
		XnUserID id,
		XnBool success,
		void* cookie )
{
	if ( success )
	{
		printf(
			"Calibration complete, start tracking user %d\n",
			id );
		user_states_[id].is_tracked_ = true;
		
		user_generator_.GetSkeletonCap().StartTracking( id );
	}
	else
	{
		printf(
			"Calibration failed for user %d\n",
			id );
		user_states_[id].is_tracked_ = false;
		
		if ( need_pose_for_calibration_ )
			user_generator_.GetPoseDetectionCap().StartPoseDetection(
				calibration_pose_,
				id );
		else
			user_generator_.GetSkeletonCap().RequestCalibration(
				id,
				TRUE );
	}
}

void XN_CALLBACK_TYPE
	poseDetectedCB(
		xn::PoseDetectionCapability& capability,
		XnChar const* strPose,
		XnUserID id,
		void* cookie )
{
    printf(
		"Pose %s detected for user %d\n",
		strPose,
		id );
	
    user_generator_.GetPoseDetectionCap().StopPoseDetection( id );
    user_generator_.GetSkeletonCap().RequestCalibration(
		id,
		TRUE );
}

void
	publishTransform(
		XnUserID const& user,
		XnSkeletonJoint const& joint,
		string const& frame_id,
		string const& child_frame_postfix )
{
	static tf::TransformBroadcaster
		br;
	
	std::stringstream child_frame_id_ss;
	child_frame_id_ss << "/user" << user << "/" << child_frame_postfix;
	
	XnSkeletonJointPosition
		joint_position;
	
	user_generator_.GetSkeletonCap().GetSkeletonJointPosition(
		user,
		joint,
		joint_position );
	
	double
		x = joint_position.position.X / 1000.0,
		y = joint_position.position.Y / 1000.0,
		z = joint_position.position.Z / 1000.0;
	
	XnSkeletonJointOrientation
		joint_orientation;
	
	user_generator_.GetSkeletonCap().GetSkeletonJointOrientation(
		user,
		joint,
		joint_orientation );
	
	XnFloat*
		m = joint_orientation.orientation.elements;
		
	KDL::Rotation 
		rotation( m[0], m[1], m[2],
				  m[3], m[4], m[5],
				  m[6], m[7], m[8] );
	
	double
		qx,
		qy,
		qz,
		qw;
	
	rotation.GetQuaternion(
		qx,
		qy,
		qz,
		qw );
	
	tf::Transform
		transform;
	
	transform.setOrigin( tf::Vector3(
		x,
		y,
		z ) );
	
	transform.setRotation( tf::Quaternion(
		qx,
		qy,
		qz,
		qw ) );
		
	double
		yaw,
		pitch,
		roll;
	transform.getBasis().getEulerZYX( yaw, pitch, roll, 1 );
//	transform.getBasis().setEulerYPR( yaw, pitch, roll - M_PI_2 );
	transform.getBasis().setEulerYPR( yaw, pitch, roll );
	
	br.sendTransform( tf::StampedTransform(
		transform,
		ros::Time::now(),
		frame_id,
		child_frame_id_ss.str() ) );
}

void
	publishTransforms(
		const std::string& frame_id )
{
	XnUserID
		users[15];
	
	XnUInt16
		users_count = 15;
	
	user_generator_.GetUsers(
		users,
		users_count );
	
	for (
		int i = 0;
		i < users_count;
		++i )
	{
		XnUserID
			user = users[i];
		
		if ( !user_generator_.GetSkeletonCap().IsTracking( user ) )
			continue;
	
	
		publishTransform( user, XN_SKEL_HEAD,           frame_id, "head" );
		publishTransform( user, XN_SKEL_NECK,           frame_id, "neck" );
		publishTransform( user, XN_SKEL_TORSO,          frame_id, "torso" );
	
		publishTransform( user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder" );
		publishTransform( user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow" );
		publishTransform( user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand" );
	
		publishTransform( user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder" );
		publishTransform( user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow" );
		publishTransform( user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand" );
	
		publishTransform( user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip" );
		publishTransform( user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee" );
		publishTransform( user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot" );
	
		publishTransform( user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip" );
		publishTransform( user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee" );
		publishTransform( user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot" );
    }
}

void publishUserStates()
{
	openni_multitracker::UserStateArray::Ptr user_states_msg( new openni_multitracker::UserStateArray );
	user_states_msg->user_states.reserve( user_states_.size() );
	
	for(
		_UserStateMap::iterator user_state = user_states_.begin();
		user_state != user_states_.end();
		++user_state )
	{
		openni_multitracker::UserState user_state_msg;
		
		std::stringstream user_state_name_ss;
		
		user_state_name_ss << "user" << user_state->first;
		
		user_state_msg.name = user_state_name_ss.str();
		user_state_msg.id = user_state->first;
		user_state_msg.is_tracked = user_state->second.is_tracked_;
		
		user_states_msg->user_states.push_back( user_state_msg );
	}
	user_states_pub_.publish( user_states_msg );
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

int
	main(
		int argc,
		char **argv )
{
    ros::init(
		argc,
		argv,
		"openni_multitracker" );
    
    ros::NodeHandle nh;

    string config_filename = ros::package::getPath( "openni_multitracker" ) + "/openni_tracker.xml";
    XnStatus nRetVal = context_.InitFromXmlFile( config_filename.c_str() );
    CHECK_RC( nRetVal, "InitFromXml" );

    nRetVal = context_.FindExistingNode( XN_NODE_TYPE_DEPTH, depth_generator_ );
    CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = context_.FindExistingNode( XN_NODE_TYPE_USER, user_generator_ );
	if ( nRetVal != XN_STATUS_OK )
	{
		nRetVal = user_generator_.Create( context_ );
		CHECK_RC( nRetVal, "Find user generator" );
	}

	if ( !user_generator_.IsCapabilitySupported( XN_CAPABILITY_SKELETON ) )
	{
		printf( "Supplied user generator doesn't support skeleton\n" );
		return 1;
	}

    XnCallbackHandle user_calbacks_handle;
	user_generator_.RegisterUserCallbacks( newUserCB, lostUserCB, NULL, user_calbacks_handle );

	XnCallbackHandle calibration_callbacks_handle;
	user_generator_.GetSkeletonCap().RegisterCalibrationCallbacks( startCalibrationCB, endCalibrationCB, NULL, calibration_callbacks_handle );

	if ( user_generator_.GetSkeletonCap().NeedPoseForCalibration() )
	{
		need_pose_for_calibration_ = true;
		
		if ( !user_generator_.IsCapabilitySupported( XN_CAPABILITY_POSE_DETECTION ) )
		{
			printf( "Pose required, but not supported\n" );
			return 1;
		}

		XnCallbackHandle pose_callbacks_handle;
		user_generator_.GetPoseDetectionCap().RegisterToPoseCallbacks( poseDetectedCB, NULL, NULL, pose_callbacks_handle );

		user_generator_.GetSkeletonCap().GetCalibrationPose( calibration_pose_ );
	}

	user_generator_.GetSkeletonCap().SetSkeletonProfile( XN_SKEL_PROFILE_ALL );

	nRetVal = context_.StartGeneratingAll();
	CHECK_RC( nRetVal, "StartGenerating" );

	ros::Rate loop_rate( 30 );

	ros::NodeHandle nh_rel( "~" );
	string frame_id;
	nh_rel.param( "camera_frame_id", frame_id, string( "openni_depth_tracking_frame" ) );
	
	user_states_pub_ = nh_rel.advertise<openni_multitracker::UserStateArray>( "user_states", 1 );
                
	while ( ros::ok() )
	{
		context_.WaitAndUpdateAll();
		publishTransforms( frame_id );
		publishUserStates();
		loop_rate.sleep();
	}

	context_.Shutdown();
	return 0;
}
