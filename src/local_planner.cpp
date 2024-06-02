#include <Eigen/Dense>
#include <ros/ros.h>
#include <potbot_lib/Utility.h>
#include <potbot_lib/DWAController.h>
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
	ros::Publisher pub_cmd_, pub_path_marker_, pub_best_path_, pub_ref_path_, pub_split_path_;
	ros::Subscriber sub_odom_;

	potbot_lib::Controller::DWAController robot_;
	nav_msgs::Odometry odom_;

	void __odom_callback(const nav_msgs::Odometry &msg);
	void __param_callback(const test_pkg::local_plannerConfig& param, uint32_t level);
public:
	LocalPlanner();
	~LocalPlanner(){};
};

LocalPlanner::LocalPlanner()
{
	ros::NodeHandle nh;
	pub_cmd_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	pub_path_marker_ = nh.advertise<visualization_msgs::MarkerArray>("multi_path", 1);
	pub_best_path_ = nh.advertise<nav_msgs::Path>("best_path", 1);
	pub_ref_path_ = nh.advertise<nav_msgs::Path>("reference_path/raw", 1);
	pub_split_path_ = nh.advertise<nav_msgs::Path>("reference_path/split", 1);

	sub_odom_ = nh.subscribe("odom", 1,&LocalPlanner::__odom_callback,this);

	dynamic_reconfigure::Server<test_pkg::local_plannerConfig> *dsrv_;
	std::string name = "";
	dsrv_ = new dynamic_reconfigure::Server<test_pkg::local_plannerConfig>(ros::NodeHandle("~/" + name));
	dynamic_reconfigure::Server<test_pkg::local_plannerConfig>::CallbackType cb = boost::bind(&LocalPlanner::__param_callback, this, _1, _2);
	dsrv_->setCallback(cb);

	std::vector<Eigen::Vector2d> reference_path;
	nav_msgs::Path reference_path_msg;
	reference_path_msg.header.frame_id = "map";
	for (double x=0; x<=5.0; x+=0.01)
	{
		reference_path.push_back(Eigen::Vector2d(x,sin(3*x)));
		geometry_msgs::PoseStamped pose;
		pose.header = reference_path_msg.header;
		pose.pose = potbot_lib::utility::get_Pose(x,sin(3*x),0,0,0,0);
		reference_path_msg.poses.push_back(pose);
	}
	robot_.set_dwa_target_path(reference_path_msg);
}

void LocalPlanner::__odom_callback(const nav_msgs::Odometry &msg)
{
	odom_ = msg;
	potbot_lib::utility::print_Pose(odom_);

	robot_.set_msg(odom_);
	robot_.calculate_command();

	geometry_msgs::Twist cmd;
	robot_.get_best_cmd(cmd);
	pub_cmd_.publish(cmd);

	nav_msgs::Path path_msg;
	robot_.get_best_path(path_msg);
	pub_best_path_.publish(path_msg);

	// pub_ref_path_.publish(reference_path_msg);

	robot_.get_split_path(path_msg);
	pub_split_path_.publish(path_msg);
	
	visualization_msgs::MarkerArray multi_path_msg;
	robot_.get_plans(multi_path_msg);
	pub_path_marker_.publish(multi_path_msg);
}

void LocalPlanner::__param_callback(const test_pkg::local_plannerConfig& param, uint32_t level)
{
	robot_.set_time_increment(param.deltatime);
	robot_.set_linear_velocity_min(param.v_min);
	robot_.set_linear_velocity_max(param.v_max);
	robot_.set_linear_velocity_increment(param.v_delta);
	robot_.set_angular_velocity_min(param.omega_min);
	robot_.set_angular_velocity_max(param.omega_max);
	robot_.set_angular_velocity_increment(param.omega_delta);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "local_planner");
	LocalPlanner lp;
	ros::spin();

    return 0;
}
