#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include "RRT.h"

using namespace std;
using namespace Eigen;

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;//地图的长宽高

// useful global variables
bool _has_map = false;

Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// ros related
ros::Subscriber _map_sub, _pts_sub;
ros::Publisher _grid_map_vis_pub, _RRTstar_path_vis_pub;

RRT* _rrt_path_finding = new RRT();

void rcvWaypointsCallback(const nav_msgs::Path &wp);

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);

void pathFinding(const Vector3d start_pt, const Vector3d target_pt);

void visRRTstarPath(vector<Vector3d> nodes);

void rcvWaypointsCallback(const nav_msgs::Path &wp) {
    if (wp.poses[0].pose.position.z < 0.0 || !_has_map)
        return;

    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");
    _rrt_path_finding->SearchPath(_start_pt, target_pt);
    auto path = _rrt_path_finding->GetPath();
    visRRTstarPath(path);
    _rrt_path_finding->Reset();
}

/*!
 * 处理地图点云数据
 * @param pointcloud_map
 */
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {
    if (_has_map)
        return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);

    if ((int) cloud.points.size() == 0) return;

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int) cloud.points.size(); idx++) {
        pt = cloud.points[idx];

        // set obstalces into grid map for path planning
        _rrt_path_finding->SetObstacle(Vector3d(pt.x, pt.y, pt.z));

        // for visualize only
        Vector3f cor_round = _rrt_path_finding->CoordRounding(Vector3d(pt.x, pt.y, pt.z)).cast<float>();

        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width = cloud_vis.points.size();
    cloud_vis.height = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "compare_demo_node");
    ros::NodeHandle nh("~");

    _map_sub = nh.subscribe("map", 1, rcvPointCloudCallBack);
    _pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallback);

    _grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _RRTstar_path_vis_pub = nh.advertise<visualization_msgs::Marker>("RRTstar_path_vis", 1);

    nh.param("map/cloud_margin", _cloud_margin, 0.0);
    nh.param("map/resolution", _resolution, 0.2);

    nh.param("map/x_size", _x_size, 50.0);
    nh.param("map/y_size", _y_size, 50.0);
    nh.param("map/z_size", _z_size, 5.0);

    nh.param("planning/start_x", _start_pt(0), 0.0);
    nh.param("planning/start_y", _start_pt(1), 0.0);
    nh.param("planning/start_z", _start_pt(2), 0.0);

    //坐标的原点在整个地图的中心
    _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0;
    _map_upper << +_x_size / 2.0, +_y_size / 2.0, _z_size;

    _inv_resolution = 1.0 / _resolution;

    //栅格地图的起点在整个地图的左下角
    _max_x_id = (int) (_x_size * _inv_resolution);
    _max_y_id = (int) (_y_size * _inv_resolution);
    _max_z_id = (int) (_z_size * _inv_resolution);

    _rrt_path_finding->InitGridMap(_map_lower, _map_upper, Vector3i(_max_x_id, _max_y_id, _max_z_id), _resolution);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    delete _rrt_path_finding;
    return 0;
}

void visRRTstarPath(vector<Vector3d> nodes) {
    visualization_msgs::Marker Points, Line;
    Points.header.frame_id = Line.header.frame_id = "world";
    Points.header.stamp = Line.header.stamp = ros::Time::now();
    Points.ns = Line.ns = "compare_demo_node/RRTstarPath";
    Points.action = Line.action = visualization_msgs::Marker::ADD;
    Points.pose.orientation.w = Line.pose.orientation.w = 1.0;
    Points.id = 0;
    Line.id = 1;
    Points.type = visualization_msgs::Marker::POINTS;
    Line.type = visualization_msgs::Marker::LINE_STRIP;

    Points.scale.x = _resolution / 2;
    Points.scale.y = _resolution / 2;
    Line.scale.x = _resolution / 2;

    //points are green and Line Strip is blue
    Points.color.g = 1.0f;
    Points.color.a = 1.0;
    Line.color.b = 1.0;
    Line.color.a = 1.0;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++) {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        Points.points.push_back(pt);
        Line.points.push_back(pt);
    }
    _RRTstar_path_vis_pub.publish(Points);
    _RRTstar_path_vis_pub.publish(Line);
}