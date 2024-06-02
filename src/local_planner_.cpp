#include <Eigen/Dense>
#include <ros/ros.h>
#include <potbot_lib/Utility.h>
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

typedef struct {
	std::vector<Eigen::Vector2d> path;
	double linear_velocity=0;
	double angular_velocity=0;
	double end_time=0;
	double delta_time=0;
} plan;

class LocalPlanner
{
private:
	double deltatime_ = 0.1;
	double v_min_=-0.2;
	double v_max_=0.2;
	double v_delta_=0.05;
	double omega_min_=-1.0;
	double omega_max_=1.0;
	double omega_delta_=0.1;

	nav_msgs::Odometry odom_;
	
	void __find_closest_vector(const std::vector<Eigen::Vector2d>& vectors, const Eigen::Vector2d& target, Eigen::Vector2d& closest);
	template <typename T>
	int __get_index(const std::vector<T>& vec, const T& value);

	void __get_reference_path(std::vector<Eigen::Vector2d>& reference_path);
	void __split_path(const std::vector<Eigen::Vector2d>& full_path, std::vector<Eigen::Vector2d>& split_path);
	void __search_for_best_plan(const std::vector<plan>& plans, const std::vector<Eigen::Vector2d>& reference_path, plan& best_plan);

	void __create_plans(std::vector<plan>& plans);
	void __plans2marker_arrary(const std::vector<plan>& plans, visualization_msgs::MarkerArray& msg);
	void __plan2path_mag(const std::vector<Eigen::Vector2d>& path, nav_msgs::Path& msg);
	void __plan2path_mag(const plan& plan, nav_msgs::Path& msg);
	void __odom_callback(const nav_msgs::Odometry &msg);
	void __param_callback(const test_pkg::local_plannerConfig& param, uint32_t level);
public:
	LocalPlanner();
	~LocalPlanner(){};
};

LocalPlanner::LocalPlanner()
{
	ros::NodeHandle nh;
	ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Publisher pub_path_marker = nh.advertise<visualization_msgs::MarkerArray>("multi_path", 1);
	ros::Publisher pub_best_path = nh.advertise<nav_msgs::Path>("best_path", 1);
	ros::Publisher pub_ref_path = nh.advertise<nav_msgs::Path>("reference_path/raw", 1);
	ros::Publisher pub_split_path = nh.advertise<nav_msgs::Path>("reference_path/split", 1);

	ros::Subscriber sub_odom = nh.subscribe("odom", 1,&LocalPlanner::__odom_callback,this);

	dynamic_reconfigure::Server<test_pkg::local_plannerConfig> *dsrv_;
	std::string name = "";
	dsrv_ = new dynamic_reconfigure::Server<test_pkg::local_plannerConfig>(ros::NodeHandle("~/" + name));
	dynamic_reconfigure::Server<test_pkg::local_plannerConfig>::CallbackType cb = boost::bind(&LocalPlanner::__param_callback, this, _1, _2);
	dsrv_->setCallback(cb);

	ros::Rate loop_rate(30);
	while (ros::ok())
	{
		std::vector<plan> plans;
		__create_plans(plans);

		std::vector<Eigen::Vector2d> reference_path;
		__get_reference_path(reference_path);
		nav_msgs::Path ref_path_msg;
		__plan2path_mag(reference_path, ref_path_msg);
		pub_ref_path.publish(ref_path_msg);

		std::vector<Eigen::Vector2d> split_reference_path;
		__split_path(reference_path, split_reference_path);
		nav_msgs::Path split_path_msg;
		__plan2path_mag(split_reference_path, split_path_msg);
		pub_split_path.publish(split_path_msg);

		plan best_plan;
		__search_for_best_plan(plans, split_reference_path, best_plan);

		nav_msgs::Path best_path_msg;
		__plan2path_mag(best_plan, best_path_msg);
		pub_best_path.publish(best_path_msg);

		visualization_msgs::MarkerArray multi_path_msg;
		__plans2marker_arrary(plans, multi_path_msg);
		pub_path_marker.publish(multi_path_msg);

		geometry_msgs::Twist cmd;
		cmd.linear.x = best_plan.linear_velocity;
		cmd.angular.z = best_plan.angular_velocity;
		pub_cmd.publish(cmd);

		ros::spinOnce();
		loop_rate.sleep();
	}

	// tf2_ros::Buffer buffer(ros::Duration(10));
	// costmap_2d::Costmap2DROS costmap("my_costmap", buffer);

	// base_local_planner::TrajectoryPlannerROS tp;
	// tp.initialize("my_trajectory_planner", &buffer, &costmap);
}

void LocalPlanner::__create_plans(std::vector<plan>& plans)
{
	plans.clear();
	double x_init = odom_.pose.pose.position.x;
	double y_init = odom_.pose.pose.position.y;
	double th_init = potbot_lib::utility::get_Yaw(odom_.pose.pose.orientation);
	double dt = deltatime_;
	double tau = 1.0;
	double v_min = v_min_;
	double v_max = v_max_;
	double v_delta = v_delta_;
	double omega_min = omega_min_;
	double omega_max = omega_max_;
	double omega_delta = omega_delta_;
	
	for (double v = v_min; v <= v_max; v += v_delta)
	{
		for (double omega = omega_min; omega <= omega_max; omega += omega_delta)
		{
			plan p;
			p.linear_velocity = v;
			p.angular_velocity = omega;
			p.delta_time = dt;
			p.end_time = tau;
			double x=x_init, y=y_init, th=th_init;
			for (double t = 0; t < tau; t += dt)
			{
				th += omega*dt;
				x += v*dt*cos(th);
				y += v*dt*sin(th);
				p.path.push_back(Eigen::Vector2d(x,y));
			}
			plans.push_back(p);
		}
	}
}

void LocalPlanner::__find_closest_vector(const std::vector<Eigen::Vector2d>& vectors, const Eigen::Vector2d& target, Eigen::Vector2d& closest) 
{
    double minDistance = std::numeric_limits<double>::max();

    for (const auto& vec : vectors) {
        double distance = (vec - target).norm();
        if (distance < minDistance) {
            minDistance = distance;
            closest = vec;
        }
    }
}

template <typename T>
int LocalPlanner::__get_index(const std::vector<T>& vec, const T& value) 
{
    auto it = std::find(vec.begin(), vec.end(), value);
    if (it != vec.end()) {
        return std::distance(vec.begin(), it);
    } else {
        return -1; // Return -1 if the element is not found
    }
}

void LocalPlanner::__get_reference_path(std::vector<Eigen::Vector2d>& reference_path)
{
	reference_path.clear();
	std::vector<Eigen::Vector2d> raw_path;
	for (double x=0; x<=5.0; x+=0.01)
	{
		reference_path.push_back(Eigen::Vector2d(x,sin(3*x)));
	}
}

void LocalPlanner::__split_path(const std::vector<Eigen::Vector2d>& full_path, std::vector<Eigen::Vector2d>& split_path)
{
	split_path.clear();
	Eigen::Vector2d closest_point;
	__find_closest_vector(full_path, Eigen::Vector2d(odom_.pose.pose.position.x, odom_.pose.pose.position.y), closest_point);
	int closest_index = __get_index(full_path, closest_point);

	double total_distance = 0;
	for (int i = closest_index; i < full_path.size(); i++)
	{
		if (i < 2)
		{
			split_path.push_back(full_path[i]);
			continue;
		}

		total_distance += (full_path[i] - full_path[i-1]).norm();
		if (total_distance < 1.0)
		{
			split_path.push_back(full_path[i]);
		}
		else
		{
			break;
		}
	}
}

void LocalPlanner::__search_for_best_plan(const std::vector<plan>& plans, const std::vector<Eigen::Vector2d>& reference_path, plan& best_plan)
{
	// best_plan = plans.back();
	// return;

	double score_min = 1e9;
	// std::vector<Eigen::Vector2d> reference_path;
	for (const auto& plan:plans)
	{
		Eigen::Vector2d s = {0,0};
		Eigen::Vector2d e = {1,0};
		size_t num_points = plan.path.size();
		if (num_points >= reference_path.size()) num_points = reference_path.size();
		// ROS_INFO("%d",num_points);
		// reference_path.clear();
		// for (size_t i = 0; i <= num_points; i++) 
		// {
		// 	double t = double(i) / double(num_points);
		// 	reference_path.push_back((1 - t) * s + t * e);
		// }

		double score_diff = 0;
		for (size_t i = 0; i < num_points; i++) 
		{
			score_diff += (plan.path[i] - reference_path[i]).norm();
		}
		// score_diff = (plan.path.back() - reference_path.back()).norm();

		if (score_diff < score_min)
		{
			best_plan = plan;
			score_min = score_diff;
		}
	}
	
}

void LocalPlanner::__plan2path_mag(const std::vector<Eigen::Vector2d>& path, nav_msgs::Path& msg)
{
	msg.poses.clear();
	msg.header.frame_id = "map";

	for (const auto& p:path)
	{
		geometry_msgs::PoseStamped pose;
		pose.header = msg.header;
		pose.pose = potbot_lib::utility::get_Pose(p(0),p(1),0,0,0,0);
		msg.poses.push_back(pose);
	}
}

void LocalPlanner::__plan2path_mag(const plan& plan, nav_msgs::Path& msg)
{
	__plan2path_mag(plan.path, msg);
}

void LocalPlanner::__plans2marker_arrary(const std::vector<plan>& plans, visualization_msgs::MarkerArray& msg)
{
	msg.markers.clear();

	visualization_msgs::Marker path_msg;
	path_msg.header.frame_id="map";
	path_msg.id = 0;
	path_msg.lifetime = ros::Duration(0.1);
	path_msg.type = visualization_msgs::Marker::LINE_STRIP;
	path_msg.pose = potbot_lib::utility::get_Pose();
	path_msg.color.a = 1;
	path_msg.color.g = 0.8;
	path_msg.scale.x = 0.001;

	for (const auto& plan:plans)
	{
		path_msg.id++;
		path_msg.points.clear();
		for (const auto& p:plan.path)
		{
			path_msg.points.push_back(potbot_lib::utility::get_Point(p(0),p(1)));
		}
		msg.markers.push_back(path_msg);
	}
}

void LocalPlanner::__odom_callback(const nav_msgs::Odometry &msg)
{
	odom_ = msg;
}

void LocalPlanner::__param_callback(const test_pkg::local_plannerConfig& param, uint32_t level)
{
	deltatime_ = param.deltatime;
	v_min_ = param.v_min;
	v_max_ = param.v_max;
	v_delta_ = param.v_delta;
	omega_delta_ = param.omega_delta;
	omega_min_ = param.omega_min;
	omega_max_ = param.omega_max;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "local_planner");
	LocalPlanner lp;
	// ros::spin();

    return 0;
}
