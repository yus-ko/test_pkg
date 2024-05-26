#include <iostream>
#include <random>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

double g_x_min = -10.0;
double g_x_max = 10.0;
double g_y_min = -10.0;
double g_y_max = 10.0;
double g_resolution = 0.1;

struct Point 
{
    double x, y, z;
    Point(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};


Eigen::VectorXd get_min_value_column(const Eigen::MatrixXd& mat) {
    // 3行目の要素を取得
    Eigen::VectorXd thirdRow = mat.row(mat.rows()-1);
	// std::cout << thirdRow << std::endl;

    // 3行目の最小値のインデックスを取得
    Eigen::Index minIndex;
    double minValue = thirdRow.minCoeff(&minIndex);
	// std::cout<< std::endl << minIndex << ":" << minValue << std::endl;

    // 最小値の列を取得
    Eigen::VectorXd minValueColumn = mat.col(minIndex);

    return minValueColumn;
}

double ackley_function(const Eigen::VectorXd& vec)
{
	size_t n = vec.size();
	double b_sum = 0;
	double d_sum = 0;
	for (size_t i = 0; i < n; i++)
	{
		b_sum += pow(vec(i),2);
		d_sum += cos(2*M_PI*vec(i));
	}
	
	double a = 20;
	double b = -20*exp(-0.2*sqrt(1.0/double(n)*b_sum));
	double c = M_E;
	double d = -exp(1.0/double(n)*d_sum);
	return a + b + c + d;
}

double rastrigin_function(const Eigen::VectorXd& vec)
{
	size_t n = vec.size();
	double b_sum = 0;
	for (size_t i = 0; i < n; i++) b_sum += (pow(vec(i),2) - 10.0*cos(2*M_PI*vec(i)));
	double a = 10.0 * double(n);
	double b = b_sum;
	return a + b;
}

double griewank_function(const Eigen::VectorXd& vec)
{
	size_t n = vec.size();
	double b_sum = 0;
	double c_sum = 1.0;
	for (size_t i = 0; i < n; i++)
	{
		b_sum += pow(vec(i),2);
		c_sum *= cos(vec(i)/sqrt(double(i)+1.0));
	}
	
	double a = 1.0;
	double b = 1.0/4000.0*b_sum;
	double c = -c_sum;
	return a + b + c;
}

double styblinski_tang_function(const Eigen::VectorXd& vec)
{
	size_t n = vec.size();
	double a_sum = 0;
	for (size_t i = 0; i < n; i++)
	{
		a_sum += pow(vec(i),4) - 16.0*pow(vec(i),2) + 5.0*vec(i);
	}
	
	double a = a_sum/2.0;
	return a;
}

double michalewicz_function(const Eigen::VectorXd& vec)
{
	size_t n = vec.size();
	double m = 10.0;
	double a_sum = 0;
	for (size_t i = 0; i < n; i++)
	{
		a_sum += sin(vec(i))*pow(sin((double(i)+1.0)*pow(vec(i),2)/M_PI),m);
	}
	double a = -a_sum;
	return a;
}

double xin_she_yang_function(const Eigen::VectorXd& vec)
{
	size_t n = vec.size();
	double a_sum = 0;
	double b_sum = 0;
	for (size_t i = 0; i < n; i++)
	{
		a_sum += abs(vec(i));
		b_sum += sin(pow(vec(i),2));
	}
	double a = a_sum*exp(-b_sum);
	return a;
}

double objective_function(const Eigen::VectorXd& vec)
{
	// return ackley_function(vec);
	// return rastrigin_function(vec);
	// return griewank_function(vec);
	// return styblinski_tang_function(vec);
	// return michalewicz_function(vec);
	return xin_she_yang_function(vec);
}

double objective_function(double x, double y)
{
	Eigen::VectorXd vec(2);
	vec << x,y;
	return objective_function(vec);
}

void create_surface(std::vector<Point>& points_output, double time = 0)
{
	points_output.clear();
	for (double x = g_x_min; x <= g_x_max; x+=g_resolution)
	{
		for (double y = g_y_min; y <= g_y_max; y+=g_resolution)
		{
			// double px = x + 11*sin(time);
			// double py = y + 11*cos(time);
			double px = x;
			double py = y;
			double pz = objective_function(px,py);
			points_output.push_back(Point(px,py,pz));
		}
	}

	// for (double r = 0; r <= sqrt(200); r+=g_resolution)
	// {
	// 	for (double th = 0; th <= 2*M_PI; th+=g_resolution)
	// 	{
	// 		double x = r*cos(th);
	// 		double y = r*sin(th);
			
	// 		double px = x + 11*sin(time);
	// 		double py = y + 11*cos(time);
	// 		double pz = objective_function(px,py);
	// 		points_output.push_back(Point(px,py,pz));
	// 	}
	// }
}

template<typename T>
void pointvec_to_pcl(const std::vector<Point>& points_input, T& pcl_output)
{
	// pclのPointCloudを作成
	T pcl_init;
	pcl_output = pcl_init;
	pcl_output.width = points_input.size();
	pcl_output.height = 1; // 単一の点群
	pcl_output.points.resize(pcl_output.width * pcl_output.height);

	// 点のベクトルをPointCloudにコピー
	for (size_t i = 0; i < points_input.size(); ++i) {
		pcl_output.points[i].x = points_input[i].x;
		pcl_output.points[i].y = points_input[i].y;
		pcl_output.points[i].z = points_input[i].z;
	}
}

void point_to_marker(const Point& points_input, visualization_msgs::Marker& marker_output)
{
	marker_output.type = visualization_msgs::Marker::SPHERE;
	marker_output.color.r = 1;
	marker_output.color.g = 1;
	marker_output.color.b = 1;
	marker_output.color.a = 1;
	marker_output.pose.position.x = points_input.x;
	marker_output.pose.position.y = points_input.y;
	marker_output.pose.position.z = points_input.z;
	marker_output.pose.orientation.x = 0;
	marker_output.pose.orientation.y = 0;
	marker_output.pose.orientation.z = 0;
	marker_output.pose.orientation.w = 1;
	marker_output.scale.x = 1.5*g_resolution;
	marker_output.scale.y = 1.5*g_resolution;
	marker_output.scale.z = 1.5*g_resolution;
}

void point_to_marker(const Eigen::VectorXd& points_input, visualization_msgs::Marker& marker_output)
{
	point_to_marker(Point(points_input(0), points_input(1), points_input(2)), marker_output);
}

void mat_to_marker_points(const Eigen::MatrixXd& points_input, visualization_msgs::Marker& marker_output)
{
	marker_output.type = visualization_msgs::Marker::SPHERE_LIST;
	marker_output.color.r = 0.1;
	marker_output.color.g = 0.1;
	marker_output.color.b = 0.1;
	marker_output.color.a = 0.7;
	marker_output.pose.position.x = 0;
	marker_output.pose.position.y = 0;
	marker_output.pose.position.z = 0;
	marker_output.pose.orientation.x = 0;
	marker_output.pose.orientation.y = 0;
	marker_output.pose.orientation.z = 0;
	marker_output.pose.orientation.w = 1;
	marker_output.scale.x = g_resolution;
	marker_output.scale.y = g_resolution;
	marker_output.scale.z = g_resolution;
	marker_output.points.clear();
	marker_output.colors.clear();
	for (size_t i = 0; i < points_input.cols(); i++)
	{
		geometry_msgs::Point p;
		p.x = points_input(0,i);
		p.y = points_input(1,i);
		p.z = objective_function(points_input(0,i), points_input(1,i));
		marker_output.points.push_back(p);
		marker_output.colors.push_back(marker_output.color);
	}
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pso");
	ros::NodeHandle nh;
	ros::Publisher pub_surface = nh.advertise<sensor_msgs::PointCloud2>("surface", 1);
	ros::Publisher pub_optimized = nh.advertise<visualization_msgs::Marker>("optimized", 1);
	ros::Publisher pub_particles = nh.advertise<visualization_msgs::Marker>("particles", 1);
	ros::Rate loop_rate(60);

	std::random_device rd;
    std::default_random_engine eng(rd());
    
	size_t params = 2;
	size_t particle_num = 100;
	size_t max_iter = 1000;
	double w = 0.6;
	double c1 = 0.25;
	double c2 = 0.25;

	std::uniform_real_distribution<double> dist_x(g_x_min, g_x_max);
	std::uniform_real_distribution<double> dist_y(g_y_min, g_y_max);
	std::uniform_real_distribution<double> dist_delta(-1.0, 1.0);
	Eigen::MatrixXd particles(params, particle_num);
	Eigen::MatrixXd velocities(params, particle_num);
	Eigen::MatrixXd pbest(params+1, particle_num);
	for (size_t i = 0; i < particle_num; i++)
	{
		Eigen::VectorXd particle_vec(params);
		Eigen::VectorXd velocity_vec(params);
		for (size_t j = 0; j < params; j++) 
		{
			particle_vec(j) = g_x_max;
			// particle_vec(j) = dist_x(eng);
			velocity_vec(j) = dist_delta(eng);
		}
		particles.col(i) << particle_vec;
		velocities.col(i) << velocity_vec;
		pbest.col(i) << particle_vec , objective_function(particles.col(i));
		// particle_vec.push_back(Point(particles.col(i)(0), particles.col(1), z));
		// velocity_vec.push_back(Point(dist_delta(eng), dist_delta(eng), dist_delta(eng)));
	}

	// Point pbest = find_min_z(particle_vec);
	Eigen::VectorXd gbest = get_min_value_column(pbest);
	// gbest = Point(g_x_max,g_y_max,objective_function(g_x_max,g_y_max));

	size_t iter = 0;
	std::vector<Point> points;
	create_surface(points,ros::Time::now().toSec());
	pcl::PointCloud<pcl::PointXYZ> pcl;
	pointvec_to_pcl(points, pcl);

	sensor_msgs::PointCloud2 pcl_msg;
	pcl::toROSMsg(pcl, pcl_msg);
	pcl_msg.header.frame_id = "map";
	while (ros::ok())
	{

		ROS_INFO("%d / %d", int(iter), int(max_iter));
		if (iter < max_iter)
		{
			for (size_t i = 0; i < particle_num; i++)
			{
				particles.col(i) = particles.col(i) + velocities.col(i);
				Eigen::VectorXd pb = pbest.block(0, i, pbest.rows()-1, 1) - particles.col(i).head(params);
				Eigen::VectorXd gb = gbest.block(0, 0, gbest.rows()-1, 1) - particles.col(i).head(params);
				velocities.col(i) = w*velocities.col(i) + c1*dist_delta(eng)*(pb) + c2*dist_delta(eng)*(gb);
				// std::cout << velocities.col(i) << std::endl;

				double pbest_value = pbest(params,i);
				double i_value = objective_function(particles.col(i));
				if (i_value < pbest_value)
				{
					pbest.col(i) << particles.col(i) , i_value;
				}
				
			}
			gbest = get_min_value_column(pbest);

			iter++;
		}
		std::cout << "Answer: " << gbest.transpose() << std::endl;

		visualization_msgs::Marker particles_marker_msg;
		mat_to_marker_points(particles, particles_marker_msg);
		particles_marker_msg.header.frame_id = "map";

		visualization_msgs::Marker optimized_marker_msg;
		point_to_marker(gbest, optimized_marker_msg);
		optimized_marker_msg.header.frame_id = "map";
		
		pub_surface.publish(pcl_msg);
		pub_particles.publish(particles_marker_msg);
		pub_optimized.publish(optimized_marker_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
