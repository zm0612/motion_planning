//
// Created by meng on 2021/8/18.
//
#ifndef GRID_PATH_SEARCH_RRTSTAR_H
#define GRID_PATH_SEARCH_RRTSTAR_H

#include "RRT_node.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <vector>

class RRT{
public:
    RRT() = default;

    ~RRT();

    void InitGridMap(const Eigen::Vector3d& xyz_coordinate_l, const Eigen::Vector3d& xyz_coordinate_u,
                     const Eigen::Vector3i& grid_size, const double& grid_resolution);

    void SetObstacle(const Eigen::Vector3d& obstacle_coord);

    /*!
     * 将点的坐标舍入到栅格中心
     * @param coord 输入的点坐标
     * @return  栅格中心的坐标
     */
    Eigen::Vector3d CoordRounding(const Eigen::Vector3d& coord);

    bool SearchPath(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt);

    std::vector<Eigen::Vector3d> GetPath();

    void Reset();

protected:
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_flann_;

    std::vector<RRTNode*> nodes_ptr_;
    Eigen::Vector3d xyz_coord_lower_;
    Eigen::Vector3d xyz_coord_upper_;
    Eigen::Vector3i grid_size_;
    double grid_resolution_;
    uint8_t *map_data_;
    int all_xyz_grid_size_;
    int all_xy_grid_size_;
    int all_x_grid_size_;

    RRTNode* end_node_;
    RRTNode* start_node_;

    Eigen::Vector3d GridIndex2Coord(const Eigen::Vector3i& index);

    Eigen::Vector3i Coord2GridIndex(const Eigen::Vector3d& point);

    Eigen::Vector3i Sample();

    RRTNode* Near(const Eigen::Vector3i& index);

    Eigen::Vector3d Steer(const Eigen::Vector3i& rand_index, const Eigen::Vector3i& near_index, double step_size);

    bool CollisionFree(const Eigen::Vector3i& near_index, const Eigen::Vector3i& new_index);

    pcl::PointCloud<pcl::PointXYZ> GetNodesIndexPointCloud();
};

#endif //GRID_PATH_SEARCH_RRTSTAR_H
