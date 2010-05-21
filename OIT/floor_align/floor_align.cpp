#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//+---------------------------------------------------------------------------------------------------+//
//|  _ camera ( c  E R^3 )                                 +-[GOAL]----------+-[LEGEND]-------------+ |//
//| [_]                                                    |             ->  |                      | |//
//| /_\               ->                                   | calculate < h > | E = 'in the set'     | |//
//|  *       normal < n  E R^3 > to ground plane           +-----------------+ * = tip of vector    | |//
//|  |\.    *              --->                                              | + = origin of vector | |//
//|  | \`.   \    normal < n_0  E R^3 > to ground plane prior to rotation    +----------------------+ |//
//|  |  \ `.t0\  *                                                                                    |//
//|  |   \t0`. \ |       ground plane                                                                 |//
//|  |  h \   `.\|   ,~'`                                                                       --->  |//
//|  |-----\----`+-'`---- center ( p  E R^3 ) of ground plane at intersetion of this line and < n_0 > |//
//|  |      +,~'`                                                                                     |//
//|  |   ,~'`       `.                                                                                |//
//|  !~'`             `.          -->                                                                 |//
//|            \        `vector < ff  E R^3 > from center of ground plane to camera                   |//
//|             \                ->                         ->                                        |//
//|              target vector < h  E R^3 > ( parallel to < n > ) from camera to ground plane         |//
//|              with length |h|, the height of the camera above the ground plane                     |//
//+---------------------------------------------------------------------------------------------------+//
/////////////////////////////////////////////////////////////////////////////////////////////////////////

tf::TransformListener * tl;
tf::TransformBroadcaster * tb;
tf::StampedTransform * raw_frame;
tf::Transform * filtered_frame;
tf::Transform * floor_norm_to_cam;

ros::Rate * loop_rate;

int numSamples;

bool getSample()
{
	static ros::Time lastTime = ros::Time(0);
	try
	{
		tl->waitForTransform("/ovh_height", "/floor", lastTime, ros::Duration(5.0));
		tl->lookupTransform("/ovh_height", "/floor", lastTime, (*raw_frame));
		
		//make sure we set a delay so we don't just grab the same frame n times
		lastTime = raw_frame->stamp_;
		lastTime += ros::Duration(0.1);
		
		fprintf(stdout, "x %f y %f z %f\n", raw_frame->getOrigin().x(), raw_frame->getOrigin().y(), raw_frame->getOrigin().z());
		
		//using filtered_frame as the sum of all the frames
		filtered_frame->setOrigin(filtered_frame->getOrigin() + raw_frame->getOrigin());
		filtered_frame->setRotation(filtered_frame->getRotation() + raw_frame->getRotation());
		
		//fprintf(stdout, "x %f y %f z %f\n", filtered_frame->getOrigin().x(), filtered_frame->getOrigin().y(), filtered_frame->getOrigin().z());
		return true;
	}   
	catch( tf::TransformException &ex )
	{
		ROS_WARN( "unable to do transformation: [%s]", ex.what());
	}
	return false;
}

void publishFilteredTransform()
{
	tb->sendTransform(tf::StampedTransform((*floor_norm_to_cam), ros::Time::now(), "/floor", "/ovh" ));
}

//wait for the specified frame and store it in raw_frame
bool lookupTransform(std::string frame1, std::string frame2)
{
	fprintf( stdout, "looking up transform from %s to %s...\n", frame1.c_str(), frame2.c_str() );
	try
	{
		tl->waitForTransform(frame1.c_str(), frame2.c_str(), ros::Time(0), ros::Duration(5.0));
		tl->lookupTransform(frame1.c_str(), frame2.c_str(), ros::Time(0), (*raw_frame));
		
		btScalar oriy, orip, orir;
		raw_frame->getBasis().getEulerYPR(oriy, orip, orir);
		
		fprintf( stdout, "found: pos <x %f y %f z %f> | ori <y %f p %f r %f>\n", raw_frame->getOrigin().x(), raw_frame->getOrigin().y(), raw_frame->getOrigin().z(), oriy, orip, orir );
		return true;
	}   
	catch( tf::TransformException &ex )
	{
		ROS_WARN( "unable to do transformation: [%s]", ex.what());
		ROS_WARN( "frame from %s to %s is not being published properly. exiting...", frame1.c_str(), frame2.c_str() );
	}
	return false;
}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "floor_align" );
	ros::NodeHandle node;
	node.param("num_samples", numSamples, 5);
	
	tl = new tf::TransformListener(node);
	tb = new tf::TransformBroadcaster();
	
	//grabbing these
	raw_frame = new tf::StampedTransform;
	
	//publishing these
	floor_norm_to_cam = new tf::Transform;
	
	//calculating these (temp use)
	filtered_frame = new tf::Transform;
	filtered_frame->setOrigin( tf::Vector3( 0, 0, 0 ) );
	
	loop_rate = new ros::Rate(100);
	
	for(int i = 0; i < numSamples; i ++)
	{
		if(!getSample())
    	{
			ROS_WARN( "frame from /floor to /ovh_height is not being published properly. exiting..." );
			return 1;
    	}
		ros::spinOnce();
	}
	
	//vector to use when averaging the origins for the sampled frames
	tf::Vector3 scaledVec ( (float)numSamples, (float)numSamples, (float)numSamples );
	
	//average the origins of the sampled frames
	filtered_frame->setOrigin(filtered_frame->getOrigin() / scaledVec);
	
	//average the rotations of the sampled frames
	filtered_frame->setRotation(filtered_frame->getRotation() / (float) numSamples);
	
	//center of the checkerboard
	tf::Vector3 * theOrigin = & ( filtered_frame->getOrigin() );
	
	fprintf(stdout, "origin: x %f y %f z %f\n", theOrigin->x(), theOrigin->y(), theOrigin->z());
	
	tf::Vector3 floorNormalVec (0, 0, 1);
	
	if( !lookupTransform("/floor", "/ovh_height") )
		return 1;
		
	tf::Vector3 floorToCam = raw_frame->getOrigin();
		
	floor_norm_to_cam->setOrigin( tf::Vector3( raw_frame->getOrigin().x(), raw_frame->getOrigin().y(), 0 ) );
	tf::Quaternion theAngle; theAngle.setEuler( 0.0f, 0.0f, 0.0f ); //only did this to kill compiler warnings
	floor_norm_to_cam->setRotation( theAngle );
	
	tb->sendTransform(tf::StampedTransform((*floor_norm_to_cam), ros::Time::now(), "/floor", "/ovh" ));
	
	float theta = floorToCam.angle( floorNormalVec );
	
	float height = filtered_frame->getOrigin().length() * cos( theta );
	
	fprintf( stdout, "theta: %f\n", theta );
	fprintf( stdout, "height: %f\n", height );
	
	if( !lookupTransform("/ovh_height", "/ovh") )
		return 1;
		
	*theOrigin = raw_frame->getOrigin();
	btScalar oriy, orip, orir;
	raw_frame->getBasis().getEulerYPR( oriy, orip, orir );
	
	fprintf(stdout, "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"floor_align_pub\" args=\"%f %f %f %f %f %f ovh_height ovh 100\" />\n", theOrigin->x(), theOrigin->y(), theOrigin->z(), oriy, orip, orir );
	
	while( node.ok() )
	{
		publishFilteredTransform();
		
		ros::spinOnce();
		loop_rate->sleep();
	}
	
	ros::spin();
	
	return 0;
}
