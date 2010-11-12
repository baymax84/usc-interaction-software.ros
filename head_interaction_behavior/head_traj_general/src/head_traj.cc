#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

class HeadTraj {

public:
	HeadTraj( ros::NodeHandle nh ) : nh_(nh)
	{	
		target_sub_ = nh_.subscribe( "head_goal", 1000, (boost::function< void(const geometry_msgs::PointStampedConstPtr&)>) boost::bind(&HeadTraj::target_cb, this, _1));
		joints_sub_ = nh_.subscribe( "joint_states", 1000, (boost::function< void(const sensor_msgs::JointStateConstPtr&)>) boost::bind(&HeadTraj::joint_cb, this, _1));

    joints_pub_ = nh_.advertise<sensor_msgs::JointState>("target_joint_state",1000);

		tl_ = new tf::TransformListener();
		//tl_->setExtrapolationLimit(ros::Duration(0.11));

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
      tl_->waitForTransform(reference_frame_, ptr->header.frame_id, ptr->header.stamp, ros::Duration(0.1) );
			tl_->transformPoint( reference_frame_, *ptr, target );
			// get pan angle

			double pan_offset = atan2( target.point.y, target.point.x );
			double tilt_offset = atan2( target.point.z, target.point.x );

      double pan_angle = 0;
      double tilt_angle = 0;

			// adjust pan and tilt link to rotate to current target
      bool pan_found = false, tilt_found = false;
      sensor_msgs::JointState target_state;

		  for( int i = 0; i < joints_.name.size(); i++ )
      {
        if( joints_.name[i] == pan_joint_name_ )
        {
          pan_found = true;
          pan_angle = joints_.position[i];
        }
        if( joints_.name[i] == tilt_joint_name_ )
        {
          tilt_found = true;
          tilt_angle = joints_.position[i];
        }
     }

      if( pan_found )
      {
        // joint value is current joint value + offset
        pan_angle += pan_offset;
        target_state.name.push_back(pan_joint_name_);
        target_state.position.push_back(pan_angle);
        target_state.velocity.push_back(0);
        target_state.effort.push_back(0);
      }

      if( tilt_found )
      {
        // joint value is current joint value + offset
        tilt_angle += tilt_offset;
        target_state.name.push_back(tilt_joint_name_);
        target_state.position.push_back(tilt_angle);
        target_state.velocity.push_back(0);
        target_state.effort.push_back(0);
      }

      target_state.header.frame_id = pan_joint_name_;
      target_state.header.stamp = ros::Time::now();
      joints_pub_.publish( target_state );

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

    ros::Publisher joints_pub_;

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
