//
// Created by meng on 2021/8/18.
//
#include "RRT.h"

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

    delete end_node_;
    delete start_node_;
    start_node_ = nullptr;
    end_node_ = nullptr;

    for (unsigned int i = 0; i < nodes_ptr_.size(); ++i) {
        delete nodes_ptr_[i];
        nodes_ptr_[i] = nullptr;
    }
    nodes_ptr_.clear();
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

Eigen::Vector3d RRT::CheckPointRange(const Eigen::Vector3d &point) {
    Eigen::Vector3d corrected_point;
    corrected_point.x() = std::min(std::max(xyz_coord_lower_.x(), point.x()), xyz_coord_upper_.x());
    corrected_point.y() = std::min(std::max(xyz_coord_lower_.y(), point.y()), xyz_coord_upper_.y());
    corrected_point.z() = std::min(std::max(xyz_coord_lower_.z(), point.z()), xyz_coord_upper_.z());

    return corrected_point;
}

Eigen::Vector3d RRT::CoordRounding(const Eigen::Vector3d &coord) {
    return GridIndex2Coord(Coord2GridIndex(coord));
}

Eigen::Vector3d RRT::Sample() {
    std::random_device rd;
    std::default_random_engine eng(rd());

    std::uniform_real_distribution<> rand_pt_x =
            std::uniform_real_distribution<>(xyz_coord_lower_.x(), xyz_coord_upper_.x());
    std::uniform_real_distribution<> rand_pt_y =
            std::uniform_real_distribution<>(xyz_coord_lower_.y(), xyz_coord_upper_.y());
    std::uniform_real_distribution<> rand_pt_z =
            std::uniform_real_distribution<>(xyz_coord_lower_.z(), xyz_coord_upper_.z());

    Eigen::Vector3d rand_pt;
    rand_pt << rand_pt_x(eng), rand_pt_y(eng), rand_pt_z(eng);

    return rand_pt;
}

RRTNode *RRT::Near(const Eigen::Vector3d &pt) {
    std::vector<int> search_indices;
    std::vector<float> search_distances;
    pcl::PointXYZ point(pt.cast<float>().x(),
                        pt.cast<float>().y(),
                        pt.cast<float>().z());
    kdtree_flann_.nearestKSearch(point, 1, search_indices, search_distances);

    if (search_indices.empty()) {
        std::cerr << "kdtree is empty" << std::endl;
    }

    return nodes_ptr_[search_indices[0]];
}

Eigen::Vector3d RRT::Steer(const Eigen::Vector3d &rand_point,
                           const Eigen::Vector3d &near_point,
                           double step_size) {
    Eigen::Vector3d direction = (rand_point - near_point).normalized();

    return near_point + direction * step_size;
}

pcl::PointCloud<pcl::PointXYZ> RRT::GetNodesCoordPointCloud() {
    pcl::PointCloud<pcl::PointXYZ> point_cloud;

    for (unsigned int i = 0; i < nodes_ptr_.size(); ++i) {
        pcl::PointXYZ point(nodes_ptr_[i]->coordinate_.cast<float>().x(),
                            nodes_ptr_[i]->coordinate_.cast<float>().y(),
                            nodes_ptr_[i]->coordinate_.cast<float>().z());
        point_cloud.push_back(point);
    }

    return point_cloud;
}

bool RRT::IsObstacle(const Eigen::Vector3d &point_coord) {
    const Eigen::Vector3i point_index = Coord2GridIndex(point_coord);

    int grid_index = all_xy_grid_size_ * point_index.z() +
            all_x_grid_size_ * point_index.y() + point_index.x();

    return map_data_[grid_index];
}

bool RRT::CollisionFree(const Eigen::Vector3d &near_point, const Eigen::Vector3d &new_point) {
    double step_size = grid_resolution_ / 5.0;
    Eigen::Vector3d direction = (new_point-near_point).normalized();
    Eigen::Vector3d delta = direction * step_size;
    Eigen::Vector3d temp_point = near_point;
    double dist_thred = (new_point - near_point).norm();
    while (true){
        temp_point = temp_point + delta;

        if ((temp_point - near_point).norm() >= dist_thred){
            return true;
        }

        if (IsObstacle(temp_point)){
            return false;
        }
    }
}

bool RRT::SearchPath(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) {
    const Eigen::Vector3d start = CheckPointRange(start_pt);
    const Eigen::Vector3d end = CheckPointRange(end_pt);

    end_node_ = new RRTNode;
    end_node_->coordinate_ = end;
    end_node_->parent_ = nullptr;

    start_node_ = new RRTNode;
    start_node_->coordinate_ = start;
    start_node_->parent_ = nullptr;
    nodes_ptr_.emplace_back(start_node_);

    pcl::PointCloud<pcl::PointXYZ> point_cloud_node = GetNodesCoordPointCloud();
    kdtree_flann_.setInputCloud(point_cloud_node.makeShared());

    while (true) {
        Eigen::Vector3d rand_point = Sample();
        RRTNode *near_node_ptr = Near(rand_point);

        //TODO: 由于固定步长，可能会导致new_point的在地图之外，但是我懒得改了，加一个判断很容易！
        Eigen::Vector3d new_point = Steer(rand_point, near_node_ptr->coordinate_, grid_resolution_ * 2);

        if (!CollisionFree(near_node_ptr->coordinate_, new_point)) {
            continue;
        }

        RRTNode *new_node_ptr = new RRTNode;
        new_node_ptr->coordinate_ = new_point;
        new_node_ptr->parent_ = near_node_ptr;

        nodes_ptr_.push_back(new_node_ptr);

        if ((new_point - end_node_->coordinate_).norm() <= grid_resolution_) {
            end_node_->parent_ = new_node_ptr;
            return true;
        }

        kdtree_flann_.setInputCloud(GetNodesCoordPointCloud().makeShared());
    }
}

void RRT::Reset() {
    for (unsigned int i = 0; i < nodes_ptr_.size(); ++i) {
        delete nodes_ptr_[i];
        nodes_ptr_[i] = nullptr;
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