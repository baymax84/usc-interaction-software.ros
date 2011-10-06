#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <wait_for_response_action/WaitForResponseAction.h>

class WaitForResponseServer {
	public:

		WaitForResponseServer( std::string name ) :
			as_(nh_, name, false),
			action_name_(name),
			goal_active_(false),
			moving_(true)

		{
      as_.registerGoalCallback(boost::bind(&WaitForResponseServer::goalCB, this));    
      as_.registerPreemptCallback(boost::bind(&WaitForResponseServer::preemptCB, this));
      as_.start();

			joint_sub_ = nh_.subscribe( "output_joint_state", 1, &WaitForResponseServer::jointCB, this);
			vel_pub_ = nh_.advertise<sensor_msgs::JointState>("vel",1);
			curr_pose_.header.stamp = ros::Time(0);


			nh_.param( "threshold", threshold_, 0.75 );
		}

		~WaitForResponseServer() {}

		void goalCB()
		{
			ROS_INFO( "%s: Goal Accepted", action_name_.c_str() );
			goal_active_ = true;
			as_.acceptNewGoal();
		}

		void preemptCB()
		{
			ROS_INFO( "%s: Preempted", action_name_.c_str() );
			as_.setPreempted();
			goal_active_ = false;
		}

		void jointCB( const sensor_msgs::JointStateConstPtr &jnt )
		{
			// is the current pose a reasonable duration from current time?
			ros::Duration time_diff = jnt->header.stamp - curr_pose_.header.stamp;

			//printf( "." ); fflush( stdout );

			if( time_diff.toSec() < 1.0 )
			{

				bool moving = false;
				for( unsigned int i = 0; i < jnt->velocity.size(); i++ )
				{
					curr_pose_.velocity[i] = fabs(jnt->position[i] - curr_pose_.position[i]) / time_diff.toSec();
					if( curr_pose_.velocity[i] > threshold_ )
					{
						//printf( "-" ); fflush( stdout );
						moving = true;
						break;
					}
					curr_pose_.header = jnt->header;
				}
				vel_pub_.publish( curr_pose_ );
				if( !moving && moving_ && goal_active_) {
					printf( "*" ); fflush( stdout );
					hold_time_ = jnt->header.stamp;
					moving_ = false;
					//ROS_INFO( "%s: holding... ", action_name_.c_str() );
				}
				else if( !moving_ && moving )
				{
					printf("-");fflush(stdout);
					hold_time_ = jnt->header.stamp;
					moving_ = true;
				}
				else
				{
					printf( "+" ); fflush(stdout);
				}
				curr_pose_.position = jnt->position;
			}
			else
			{
				moving_ = true;
				curr_pose_ = *jnt;
			}
		
			if( !moving_ )
			{
				// get duration
				ros::Duration hold_duration = jnt->header.stamp - hold_time_;

				// if over time thresh and goal is active, send succeeded
				if( hold_duration.toSec() > 1.5 && goal_active_ )
				{
					ROS_INFO( "%s: pose detected", action_name_.c_str() );
					wait_for_response_action::WaitForResponseResult res;
					res.pose = curr_pose_;
          printf ("\n");
          for( int i = 0; i < 8; i++ )
          {
            printf( "%0.2f ", res.pose.position[i] );
          }
          printf ("\n");

					as_.setSucceeded(res);

					// set goal to no longer active
					goal_active_ = false;
					moving_ = true;
				}


			}

			// update curr_pose_
			curr_pose_ = *jnt;
		}

	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<wait_for_response_action::WaitForResponseAction> as_;
		std::string action_name_;
		ros::Subscriber joint_sub_;
		ros::Publisher vel_pub_;
		bool goal_active_;

		bool moving_;
		double threshold_;
		sensor_msgs::JointState curr_pose_;
		ros::Time hold_time_;
};

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "wait_for_response_server" );
	WaitForResponseServer wfrs(ros::this_node::getName());
	ros::spin();
	return 0;
}
