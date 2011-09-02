#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <pose_action/PoseAction.h>
#include <pose_action/PoseActionGoal.h>

#define DTOR( a ) (M_PI * (a) / 180.0)

class PoseActionServer {

	public:
		PoseActionServer(std::string name) :
			as_(nh_, name, false),
			action_name_(name),
			action_running_(false),
			joints_received_(false),
			going_home_(false),
			waiting_(false)
		{
			as_.registerGoalCallback(boost::bind(&PoseActionServer::goalCB, this));			
			as_.registerPreemptCallback(boost::bind(&PoseActionServer::preemptCB, this));			
			as_.start();

			sub_ = nh_.subscribe("bandit_joint_state",1, &PoseActionServer::jointCB, this);
			target_pub_ = nh_.advertise<sensor_msgs::JointState>("target_joints",1);
		}

		~PoseActionServer()
		{

		}

		void jointCB( const sensor_msgs::JointStateConstPtr &js )
		{
			curr_pose_ = *js;
			if( !joints_received_ ) joints_received_ = true;
			printf( "." ); fflush(stdout);
		}

		void resetToHome()
		{
			sensor_msgs::JointState end_pose;

			end_pose.name = curr_pose_.name;
			end_pose.velocity = curr_pose_.velocity;
			end_pose.effort = curr_pose_.effort;

			for( unsigned int i = 0; i < curr_pose_.name.size(); i++ )
			{
				end_pose.position.push_back(0);
			}

			end_pose.position[3] = DTOR(20);
			end_pose.position[10] = DTOR(20);

			start_pose_ = curr_pose_;
			start_time_ = ros::Time::now();
			goal_pose_ = end_pose;
			goal_duration_ = ros::Duration(2.0);
			going_home_ = true;
			waiting_ = false;
		}

		void goalCB()
		{
			goal_ptr_ = as_.acceptNewGoal();		
			ROS_INFO( "%s: Goal Accepted", action_name_.c_str() );

			// record start pose
			while( ros::ok() && !joints_received_ )
			{
				ros::spinOnce();
				usleep(10 * 1000);
			}

			ROS_INFO( "%s: Goal started", action_name_.c_str() );

			start_time_ = ros::Time::now();
			start_pose_ = curr_pose_;

			// record start time

			goal_duration_ = goal_ptr_->move_duration;
			wait_duration_ = goal_ptr_->pose_duration;
			goal_pose_ = goal_ptr_->goal_state;
			action_running_ = true;

		}

		void preemptCB()
		{
			ROS_INFO("%s: Preempted", action_name_.c_str());
			as_.setPreempted();
		}

		void spin()
		{
			ros::Rate loop_rate(10);		

			while( ros::ok() )
			{
				// determine progress percentage (dur / diff)
				ros::Duration elapsed = ros::Time::now() - start_time_;
				double progress = elapsed.toSec() / goal_duration_.toSec();

				if( progress >= 1.0 && (action_running_ || going_home_ || waiting_) )
				{
					pose_action::PoseResult res;
					// if finished

					action_running_ = false;
					if( !waiting_ && !going_home_)
					{
						waiting_start_time_ = ros::Time::now();
						as_.setSucceeded(res);
						waiting_ = true;
					}
					bool waiting_done = (ros::Time::now() - waiting_start_time_) > wait_duration_;
					//ROS_INFO( "%0.2d < %0.2d", (ros::Time::now()-waiting_start_time_).toSec(), wait_duration_.toSec() );
					if( !going_home_ && (waiting_ && waiting_done) )
					{
						ROS_INFO( "%s: Goal Finished", action_name_.c_str() );

						resetToHome();
						progress = 0.0; // needed to keep arm from moving to goal at start of going home
					}
					else
						going_home_ = false;

				}
				if( action_running_ || going_home_ )
				{
					sensor_msgs::JointState target_pose;

					// for each joint in goal_pose_
					for( int i = 0; i < goal_pose_.name.size(); i++ )
					{
						int idx = -1;
						// find corresponding joint in start_pose
						for( int j = 0; j < curr_pose_.name.size(); j++ )
						{
							if( goal_pose_.name[i] == curr_pose_.name[j] )
							{
								idx = j;
								break;
							}
						}

						if( idx < 0 )
						{
							ROS_WARN( "joint (%s) not found in curr_pose", goal_pose_.name[i].c_str() );
							continue;
						}

						// set joint_state msg start + progress * diff
						double diff = goal_pose_.position[i] - start_pose_.position[idx];
						double set = diff * progress;

						target_pose.name.push_back( goal_pose_.name[i] );
						target_pose.position.push_back( start_pose_.position[idx] + set );
						target_pose.velocity.push_back(0);
						target_pose.effort.push_back(0);
						//ASSUMPTION, that start pose and curr pose indices are aligned, true for bandit
					}

					target_pub_.publish(target_pose);

					// send feedback msga
					if( !action_running_ )
					{
						pose_action::PoseFeedback feedback;
						feedback.progress = progress;
						as_.publishFeedback( feedback );
					}
				}

				loop_rate.sleep();
				ros::spinOnce();
			}
		}

	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<pose_action::PoseAction> as_;
		std::string action_name_;
		ros::Subscriber sub_;
		ros::Publisher target_pub_;


		boost::shared_ptr<const pose_action::PoseGoal> goal_ptr_;
		ros::Time start_time_;
		ros::Time waiting_start_time_;
		ros::Duration wait_duration_;
		ros::Duration goal_duration_;

		sensor_msgs::JointState start_pose_;
		sensor_msgs::JointState goal_pose_;
		sensor_msgs::JointState curr_pose_;

		bool action_running_;
		bool joints_received_;
		bool going_home_;
		bool waiting_;
};

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "pose_action_server" );
	PoseActionServer server(ros::this_node::getName());
	server.spin();
	return 0;
}

