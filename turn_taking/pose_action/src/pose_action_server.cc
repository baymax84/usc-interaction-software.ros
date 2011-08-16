#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <pose_action/PoseAction.h>
#include <pose_action/PoseActionGoal.h>
class PoseActionServer {

	public:
		PoseActionServer(std::string name) :
			as_(nh_, name, false),
			action_name_(name)
		{
			as_.registerGoalCallback(boost::bind(&PoseActionServer::goalCB, this));			
			as_.registerPreemptCallback(boost::bind(&PoseActionServer::preemptCB, this));			
			as_.start();

			//subscribe to joint states to get current state of joints

		}

		~PoseActionServer()
		{

		}


		void goalCB()
		{
			boost::shared_ptr<PoseGoal> goal_ptr =	as_.acceptNewGoal();
			ROS_INFO( "%s: Goal Accepted", action_name.c_str() );
			pose_action::PoseActionResult res;
			as_->setSuceeded(res);
		}

		void preemptCB()
		{
			ROS_INFO("%s: Preempted", action_name.c_str());
			as_.setPreempted();
		}

	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<pose_action::PoseAction> as_;
		std::string action_name_;
		ros::Subscriber sub_;
};

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "pose_action_server" );
	PoseActionServer server(ros::this_node::getName());
	ros::spin();
	return 0;
}

