#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "mesh_visualization_node");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "mesh_visualization";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // marker.scale.x = 0.001;
    // marker.scale.y = 0.001;
    // marker.scale.z = 0.001;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    // // メッシュリソースのパスを指定（例：package://mesh_visualization/meshes/mesh.stl）
    // marker.mesh_resource = "turtlebot3_description://meshes/bases/burger_base.stl";
    // marker.mesh_resource = "/home/ros/catkin_ws/src/turtlebot3/turtlebot3_description/meshes/bases/burger_base.stl";
    // marker.mesh_resource = "/home/ros/catkin_ws/src/turtlebot3/turtlebot3_description/meshes/sensors/lds.stl";
    // marker.mesh_resource = "package://turtlebot3_description/meshes/bases/burger_base.stl";
    marker.mesh_resource = "package://turtlebot3_description/meshes/sensors/astra.dae";

    ros::Rate r(30);
    while (ros::ok()) {
        marker_pub.publish(marker);
        r.sleep();
    }

    return 0;
}