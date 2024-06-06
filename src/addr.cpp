#include <Eigen/Dense>
#include <ros/ros.h>
#include <potbot_lib/Utility.h>
#include <sensor_msgs/LaserScan.h>
// #include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <test_pkg/local_plannerConfig.h>

class LocalPlanner
{
private:
	ros::Subscriber sub_odom_;
	sensor_msgs::LaserScan* scan_;

	void __scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
public:
	LocalPlanner();
	~LocalPlanner(){};
};

LocalPlanner::LocalPlanner()
{
	ros::NodeHandle nh;
	// sub_odom_ = nh.subscribe("scan", 1,&LocalPlanner::__scan_callback,this);

	sensor_msgs::LaserScan scan;
	scan.ranges.resize(1000);
	while (ros::ok())
	{
		ros::Time begin = ros::Time::now();
		for (size_t i = 0; i < 1000; i++)
		{
			for (auto& p:scan.ranges)
			{
				p=10325.0*32985791.0/32586989.0;
			}
		}
		ros::Time end = ros::Time::now();
		ROS_INFO("%f", end.toSec() - begin.toSec());
	}
	
}

void LocalPlanner::__scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	ROS_INFO("scan callback");
	// scan_ = *msg;
	sensor_msgs::LaserScan scan;
	scan_ = &scan;
	// ROS_INFO("addr: %s",&msg);
	std::cout << "変数のアドレス: " << msg << std::endl;
	std::cout << "変数のアドレス: " << &scan_ << std::endl;

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "addr_test");
	LocalPlanner lp;
	ros::spin();

    return 0;
}
