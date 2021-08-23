//
// Created by meng on 2021/8/18.
//
#include "RRT.h"

#include <cmath>
#include <random>
#include <iostream>

void RRT::InitGridMap(const Eigen::Vector3d &xyz_coordinate_lower, const Eigen::Vector3d &xyz_coordinate_upper,
                      const Eigen::Vector3i &grid_size, const double &grid_resolution) {
    xyz_coord_lower_ = xyz_coordinate_lower;
    xyz_coord_upper_ = xyz_coordinate_upper;
    grid_size_ = grid_size;
    grid_resolution_ = grid_resolution;

    all_x_grid_size_ = grid_size[0];
    all_xy_grid_size_ = grid_size[0] * grid_size[1];
    all_xyz_grid_size_ = grid_size[0] * grid_size[1] * grid_size[2];
    map_data_ = new uint8_t[all_xyz_grid_size_];
    memset(map_data_, 0, all_xyz_grid_size_ * sizeof(uint8_t));
}

void RRT::SetObstacle(const Eigen::Vector3d &obstacle_coord) {
    if (obstacle_coord.x() < xyz_coord_lower_.x() || obstacle_coord.x() >= xyz_coord_upper_.x() ||
        obstacle_coord.y() < xyz_coord_lower_.y() || obstacle_coord.y() >= xyz_coord_upper_.y() ||
        obstacle_coord.z() < xyz_coord_lower_.z() || obstacle_coord.z() >= xyz_coord_upper_.z()
            ) {
        return;
    }

    Eigen::Vector3i current_grid_coord = ((obstacle_coord - xyz_coord_lower_) / grid_resolution_).cast<int>();
    const int index = current_grid_coord.z() * all_xy_grid_size_
                      + current_grid_coord.y() * all_x_grid_size_
                      + current_grid_coord.x();
    map_data_[index] = 1;
}

RRT::~RRT() {
    delete[] map_data_;
    map_data_ = nullptr;
}

Eigen::Vector3d RRT::GridIndex2Coord(const Eigen::Vector3i &index) {
    Eigen::Vector3d point;

    point.x() = ((double) index.x() + 0.5) * grid_resolution_ + xyz_coord_lower_.x();
    point.y() = ((double) index.y() + 0.5) * grid_resolution_ + xyz_coord_lower_.y();
    point.z() = ((double) index.z() + 0.5) * grid_resolution_ + xyz_coord_lower_.z();

    return point;
}

Eigen::Vector3i RRT::Coord2GridIndex(const Eigen::Vector3d &point) {
    Eigen::Vector3i index;
    index.x() = std::min(std::max(int((point.x() - xyz_coord_lower_.x()) / grid_resolution_), 0), grid_size_.x() - 1);
    index.y() = std::min(std::max(int((point.y() - xyz_coord_lower_.y()) / grid_resolution_), 0), grid_size_.y() - 1);
    index.z() = std::min(std::max(int((point.z() - xyz_coord_lower_.z()) / grid_resolution_), 0), grid_size_.z() - 1);

    return index;
}

Eigen::Vector3d RRT::CoordRounding(const Eigen::Vector3d &coord) {
    return GridIndex2Coord(Coord2GridIndex(coord));
}

Eigen::Vector3i RRT::Sample() {
    std::random_device rd;
    std::default_random_engine eng(rd());

    std::uniform_int_distribution<int> rand_index_x = std::uniform_int_distribution<int>(0, grid_size_.x() - 1);
    std::uniform_int_distribution<int> rand_index_y = std::uniform_int_distribution<int>(0, grid_size_.y() - 1);
    std::uniform_int_distribution<int> rand_index_z = std::uniform_int_distribution<int>(0, grid_size_.z() - 1);

    Eigen::Vector3i rand_index;
    rand_index << rand_index_x(eng), rand_index_y(eng), rand_index_z(eng);

    return rand_index;
}

RRTNode *RRT::Near(const Eigen::Vector3i &index) {
    std::vector<int> search_indices;
    std::vector<float> search_distances;
    pcl::PointXYZ point(index.cast<float>().x(),
                        index.cast<float>().y(),
                        index.cast<float>().z());
    kdtree_flann_.nearestKSearch(point, 1, search_indices, search_distances);

    if (search_indices.empty()) {
        std::cerr << "kdtree is empty" << std::endl;
    }

    return nodes_ptr_[search_indices[0]];
}

Eigen::Vector3d RRT::Steer(const Eigen::Vector3i &rand_index,
                           const Eigen::Vector3i &near_index,
                           double step_size) {
//    Eigen::Vector3d direction = (rand_index.cast<double>() - near_index.cast<double>()).normalized();
//    return direction * step_size + near_index.cast<double>();
    return rand_index.cast<double>();
}

pcl::PointCloud<pcl::PointXYZ> RRT::GetNodesIndexPointCloud() {
    pcl::PointCloud<pcl::PointXYZ> point_cloud;

    for (unsigned int i = 0; i < nodes_ptr_.size(); ++i) {
        pcl::PointXYZ point(nodes_ptr_[i]->index_.cast<float>().x(),
                            nodes_ptr_[i]->index_.cast<float>().y(),
                            nodes_ptr_[i]->index_.cast<float>().z());
        point_cloud.push_back(point);
    }

    return point_cloud;
}

bool RRT::CollisionFree(const Eigen::Vector3i &near_index, const Eigen::Vector3i &new_index) {
    return true;
}

bool RRT::SearchPath(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) {
    const Eigen::Vector3i start_index = Coord2GridIndex(start_pt);
    const Eigen::Vector3i end_index = Coord2GridIndex(end_pt);

    end_node_ = new RRTNode;
    end_node_->coordinate_ = end_pt;
    end_node_->index_ = end_index;
    end_node_->parent_ = nullptr;

    start_node_ = new RRTNode;
    start_node_->coordinate_ = start_pt;

    start_node_->index_ = start_index;
    start_node_->parent_ = nullptr;
    nodes_ptr_.emplace_back(start_node_);

    pcl::PointCloud<pcl::PointXYZ> point_cloud_node = GetNodesIndexPointCloud();
    kdtree_flann_.setInputCloud(point_cloud_node.makeShared());

    while (true) {
//    for (int i = 0; i < 10; ++i) {
        std::cout << "searching ..." << std::endl;
        std::cout << "end index: " << end_node_->index_.transpose() << std::endl;
        std::cout << "nodes size: " << nodes_ptr_.size() << std::endl;
        Eigen::Vector3i rand_index = Sample();
        std::cout << "rand index: " << rand_index.transpose() << std::endl;
        RRTNode *near_node_ptr = Near(rand_index);

        std::cout << "near index: " << near_node_ptr->index_.transpose() << std::endl;
        double delta_dist = (rand_index - near_node_ptr->index_).cast<double>().norm();
//        if (delta_dist * grid_resolution_ < 0.5) {
//            continue;
//        }

        Eigen::Vector3d new_pt_index = Steer(rand_index, near_node_ptr->index_, 0.5);
        std::cout << "new index: " << new_pt_index.transpose() << std::endl;
        Eigen::Vector3d new_pt_coord = GridIndex2Coord(new_pt_index.cast<int>());
        std::cout << "new pt: " << new_pt_coord.transpose() << std::endl << std::endl << std::endl;

        if (!CollisionFree(near_node_ptr->index_, new_pt_index.cast<int>())) {
            continue;
        }

        RRTNode *new_node_ptr = new RRTNode;
        new_node_ptr->index_ = new_pt_index.cast<int>();
        new_node_ptr->coordinate_ = new_pt_coord;
        new_node_ptr->parent_ = near_node_ptr;

        nodes_ptr_.push_back(new_node_ptr);

        if ((new_pt_coord - end_node_->coordinate_).norm() <= grid_resolution_) {
            end_node_->parent_ = new_node_ptr;
            return true;
        }

        kdtree_flann_.setInputCloud(GetNodesIndexPointCloud().makeShared());
    }

    return false;
}

void RRT::Reset() {
    for (unsigned int i = 0; i < nodes_ptr_.size(); ++i) {
        delete nodes_ptr_[i];
    }
    nodes_ptr_.clear();
}

std::vector<Eigen::Vector3d> RRT::GetPath() {
    std::vector<Eigen::Vector3d> path;

    RRTNode *node_ptr = end_node_;
    while (node_ptr != nullptr) {
        path.emplace_back(node_ptr->coordinate_);
        node_ptr = node_ptr->parent_;
    }

    std::reverse(path.begin(), path.end());

    return path;
}
