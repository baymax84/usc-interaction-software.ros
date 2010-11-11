#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

class HeadTraj {

public:
	HeadTraj( ros::NodeHandle nh ) : nh_(nh)
	{	
		target_sub_ = nh_.subscribe( "head_goal", 1000, (boost::function< void(const geometry_msgs::PointStampedConstPtr&)>) boost::bind(&HeadTraj::target_cb, this, _1));
		joints_sub_ = nh_.subscribe( "joint_states", 1000, (boost::function< void(const sensor_msgs::JointStateConstPtr&)>) boost::bind(&HeadTraj::joint_cb, this, _1));
		tl_ = new tf::TransformListener();
		tl_->setExtrapolationLimit(ros::Duration(1.0));

		nh_.param( "reference_frame", reference_frame_, std::string("bandit_head_tilt_link"));
		nh_.param( "pan_joint_name", pan_joint_name_, std::string("bandit_head_pan_joint"));
		nh_.param( "tilt_joint_name", tilt_joint_name_, std::string("bandit_head_tilt_joint"));
	}

	~HeadTraj()
	{
	}	

	void target_cb( const geometry_msgs::PointStampedConstPtr &ptr )
	{
		// transform point to targeting frame	
		geometry_msgs::PointStamped target;
		try {
			tl_->transformPoint( reference_frame_, *ptr, target );
			// get pan angle

			double pan_angle = atan2( target.point.y, target.point.x );
			double tilt_angle = atan2( target.point.z, target.point.x );

			// adjust pan link to rotate to current target
		
			ROS_INFO( "pan: %0.2f tilt: %0.2f", pan_angle, tilt_angle );
		}
		catch( tf::TransformException e )
		{
			ROS_WARN("unable to do transform: [%s]", e.what() );
		}
	}

	void joint_cb( const sensor_msgs::JointStateConstPtr &js )
	{
		joints_ = *js;
	}

	protected:

		ros::NodeHandle nh_;
		ros::Subscriber target_sub_;
		ros::Subscriber frame_sub_;
		ros::Subscriber joints_sub_;
		tf::TransformListener *tl_;

		std::string target_frame_;
		std::string reference_frame_;

		std::string pan_joint_name_;
		std::string tilt_joint_name_;

		sensor_msgs::JointState joints_;

};

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "head_traj" );
	ros::NodeHandle nh;
	HeadTraj traj(nh);

	ros::spin();

	return 0;
}
