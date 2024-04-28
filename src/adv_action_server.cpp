#include <ros/ros.h>
#include <potbot_lib/Utility.h>
#include <test_pkg/NavigationAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<test_pkg::NavigationAction> Server;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "task_server");
	ros::NodeHandle nh;
	Server server(nh, "task", false);
	server.start();

	ros::Time start_time;
	ros::Rate loop_rate(2);
	test_pkg::NavigationGoalConstPtr current_goal;
	while (ros::ok())
	{
		if (server.isNewGoalAvailable())
		{
			current_goal = server.acceptNewGoal();
			start_time = ros::Time::now();
			printf("Update Goal\n");
			potbot_lib::utility::print_Pose(current_goal->goal_pose.pose);
		}
		if (server.isActive())
		{
			if (server.isPreemptRequested())
			{
				server.setPreempted();
				printf("Preempt Goal\n");
			}
			else
			{
				if (start_time + ros::Duration(5) < ros::Time::now())
				{
					server.setSucceeded();
					// server.setAborted();
					printf("Active: publish result \n");
				}
				else
				{
					double t = ros::Time::now().toSec() - start_time.toSec();
					
					test_pkg::NavigationFeedback feedback;
					feedback.odom.pose.pose = potbot_lib::utility::get_Pose(t, t, 0, 0, 0, t);
					potbot_lib::utility::print_Pose(feedback.odom.pose.pose);
					server.publishFeedback(feedback);
				}
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
