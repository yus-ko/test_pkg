#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <dynamic_reconfigure/server.h>
#include <test_pkg/pcl_figConfig.h>

double g_wave_time_frequency = 0.2;
double g_wave_frequency = 1;
double g_wave_amplitude = 1;
double g_circle_wave_time_frequency = 0.2;
double g_circle_wave_frequency = 1/8;
double g_circle_wave_distance = 1;
double g_circle_wave_amplitude = 0.2;

struct Point 
{
    double x, y, z;
    Point(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

// 球面座標を直交座標に変換する関数
Point sphericalToCartesian(double r, double theta, double phi) 
{
    double x = r * sin(theta) * cos(phi);
    double y = r * sin(theta) * sin(phi);
    double z = r * cos(theta);
    return Point(x, y, z);
}

void create_sphere(std::vector<Point>& points_output, double time = 0)
{
	points_output.clear();
	size_t numPoints = 10000; // 点の数
	// 球面座標系で点を生成して配列に格納
	for (int i = 0; i < numPoints; ++i) {
		double theta = acos(-1.0 + (2.0 * i) / numPoints); // θは[0, π]の範囲になる
		double phi = sqrt(numPoints * M_PI) * theta; // φは[0, 2π]の範囲になる
		Point p = sphericalToCartesian(sin(time), theta, phi);
		// p.x += sin(time);
		// p.z += cos(time);
		points_output.push_back(p);
	}
}

void create_wave_2D(std::vector<Point>& points_output, double time = 0)
{
	points_output.clear();
	for (double x = 0; x <= 2*M_PI; x+= 0.01)
	{
		double px = x;
		double py = 0;
		double pz = sin(x + time);
		points_output.push_back(Point(px,py,pz));
	}
}

void create_wave_3D(std::vector<Point>& points_output, double time = 0)
{
	points_output.clear();
	double r = 1;
	for (double x = 0; x <= 2*M_PI; x+= 0.05)
	{
		for (double theta = 0; theta <= 2*M_PI; theta+= 0.05)
		{
			double px = x;
			double py = r * sin(theta);
			double pz = r * cos(theta) + sin(x + time);
			points_output.push_back(Point(px,py,pz));
		}
	}
}

void create_wave_circle(std::vector<Point>& points_output, double time = 0)
{
	points_output.clear();
	for (double theta = 0; theta <= 2*M_PI; theta+= 0.01)
	{
		double r = g_circle_wave_distance + g_circle_wave_amplitude*sin((1/g_circle_wave_frequency)*theta);
		double px = r*cos(theta+time);
		double py = r*sin(theta+time);
		double pz = 0;
		points_output.push_back(Point(px,py,pz));
	}
}

void create_wave_circle_3D(std::vector<Point>& points_output, double time = 0)
{
	points_output.clear();
	for (double x = 0; x <= 2*M_PI; x+= 0.05)
	{
		for (double theta = 0; theta <= 2*M_PI; theta+= 0.05)
		{
			double r = g_circle_wave_distance + g_circle_wave_amplitude*sin((1/g_circle_wave_frequency)*theta);
			double px = x;
			double py = r * sin(theta + 2*M_PI*g_circle_wave_time_frequency*time);
			double pz = r * cos(theta + 2*M_PI*g_circle_wave_time_frequency*time) + g_wave_amplitude*sin((1/g_wave_frequency)*x + 2*M_PI*g_wave_time_frequency*time);
			points_output.push_back(Point(px,py,pz));
		}
	}
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

void param_callback(const test_pkg::pcl_figConfig& param, uint32_t level)
{
	g_wave_time_frequency = param.wave_time_frequency;
	g_wave_frequency = param.wave_frequency;
	g_wave_amplitude = param.wave_amplitude;
	g_circle_wave_time_frequency = param.circle_wave_time_frequency;
	g_circle_wave_frequency = param.circle_wave_frequency;
	g_circle_wave_distance = param.circle_wave_distance;
	g_circle_wave_amplitude = param.circle_wave_amplitude;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcl_figure");
	ros::NodeHandle nh;
	ros::Publisher pub_pc = nh.advertise<sensor_msgs::PointCloud2>("pcl_figure", 1);
	ros::Rate loop_rate(30);

	dynamic_reconfigure::Server<test_pkg::pcl_figConfig> server;
	dynamic_reconfigure::Server<test_pkg::pcl_figConfig>::CallbackType f;
	f = boost::bind(param_callback, _1, _2);
	server.setCallback(f);

	while (ros::ok())
	{

		std::vector<Point> points;
		create_wave_circle_3D(points, ros::Time::now().toSec());

		pcl::PointCloud<pcl::PointXYZ> pcl;
		pointvec_to_pcl(points, pcl);

		// sensor_msgs::PointCloud2に変換
		sensor_msgs::PointCloud2 pcl_msg;
		pcl::toROSMsg(pcl, pcl_msg);
		pcl_msg.header.frame_id = "map"; // フレームIDを設定

		pub_pc.publish(pcl_msg);
		ros::spinOnce();
		loop_rate.sleep(); // ループの周期を調整
	}
	

    

    return 0;
}
