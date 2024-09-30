#include <potbot_lib/utility_ros.h>
#include <eigen3/Eigen/Dense>

using namespace potbot_lib;
using namespace potbot_lib::utility;

void printPose(Eigen::Affine3d aff)
{
	print_pose(get_pose(aff));
}

void printPose(Pose pose)
{
	printPose(pose.to_affine());
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"affine_node");
	ros::NodeHandle pnh("~");

	std::string frame_id_world = "map";
	std::string frame_id_odom = "odom";
	std::string frame_id_sensor = "sensor";
	std::string frame_id_marker = "marker";

	Eigen::Affine3d a_marker_world; //世界座標系マーカー（既知）
	Eigen::Affine3d a_sensor_world; //世界座標系センサー（未知）
	Eigen::Affine3d a_marker_sensor; //センサー座標系マーカー（既知）
	Eigen::Affine3d a_sensor_pose_from_marker; //マーカー観測により求めたセンサー位置（世界座標系）
	Eigen::Affine3d a_sensor_pose_from_encoder; //ホイールエンコーダーにより求めたセンサー位置（世界座標系）
	
	// tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
	Pose p_marker_world = {2,1,0,0,0,M_PI};
	a_marker_world = p_marker_world.to_affine();
	// ROS_INFO_STREAM(a_marker_world.translation() << a_marker_world.rotation());

	// Pose p_sensor_world = {1,1,0,0,M_PI_2,-M_PI_2};
	// a_sensor_world = p_sensor_world.to_affine();

	// Pose p_marker_sensor = {0,0,1,-M_PI_4,M_PI_2,0};
	// a_marker_sensor = p_marker_sensor.to_affine();

	// Pose pt_marker_sensor = {0,0,1,0,0,0};
	// Pose pr_marker_sensor = {0,0,0,-M_PI_4,M_PI_2,0};
	// a_marker_sensor = pr_marker_sensor.to_affine()*pt_marker_sensor.to_affine();

	Pose p_sensor_world = {1,1,0,0,0,0};
	a_sensor_world = p_sensor_world.to_affine();

	Pose p_marker_sensor = {3,-1.5,0,0,0,M_PI};
	a_marker_sensor = p_marker_sensor.to_affine();

	// a_marker_world = a_marker_sensor*a_sensor_world;
	a_sensor_world = a_marker_world*a_marker_sensor.inverse();
	// a_marker_sensor = a_sensor_world.inverse()*a_marker_world;
	// ROS_INFO("marker pose from sensor");
	// printPose(a_marker_sensor);

	// Pose p_sensor_pose_from_marker = {-0.5,1.5,0,0,0,0};
	// a_sensor_pose_from_marker = p_sensor_pose_from_marker.to_affine();

	Pose p_sensor_pose_from_encoder = {-1,3,0,0,0,0};
	a_sensor_pose_from_encoder = p_sensor_pose_from_encoder.to_affine();

	Eigen::Vector3d v_translation_diff = a_sensor_world.translation() - a_sensor_pose_from_encoder.translation();
	Pose p_translation_diff = {v_translation_diff.x(),v_translation_diff.y(),v_translation_diff.z(),0,0,0};
	printPose(p_translation_diff);

	tf2_ros::TransformBroadcaster dynamic_br;
	ros::Rate rate(60);
	while (ros::ok())
	{
		potbot_lib::utility::broadcast_frame(dynamic_br, frame_id_world, frame_id_odom, get_pose(p_translation_diff));
		potbot_lib::utility::broadcast_frame(dynamic_br, frame_id_odom, frame_id_sensor, get_pose(a_sensor_pose_from_encoder));
		// potbot_lib::utility::broadcast_frame(dynamic_br, frame_id_sensor, frame_id_marker, get_pose(a_marker_sensor));
		potbot_lib::utility::broadcast_frame(dynamic_br, frame_id_world, frame_id_marker, get_pose(a_marker_world));
		// potbot_lib::utility::broadcast_frame(dynamic_br, frame_id_world, frame_id_sensor, get_pose(a_sensor_world));
		
		ros::spinOnce();
		rate.sleep();
	}
			
	return 0;
}