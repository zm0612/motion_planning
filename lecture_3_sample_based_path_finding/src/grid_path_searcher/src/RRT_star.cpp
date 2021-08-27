//
// Created by meng on 2021/8/26.
//
#include "RRT_star.h"

RRTStar::~RRTStar() {
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

bool RRTStar::SearchPath(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) {
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

        Eigen::Vector3d new_point = Steer(rand_point, near_node_ptr->coordinate_, grid_resolution_ * 2);

        if (!CollisionFree(near_node_ptr->coordinate_, new_point)) {
            continue;
        }


        RRTNode *new_node_ptr = new RRTNode;
        new_node_ptr->coordinate_ = new_point;

        if ((new_point - end_node_->coordinate_).norm() <= grid_resolution_) {
            new_node_ptr->parent_ = near_node_ptr;
            end_node_->parent_ = new_node_ptr;
            return true;
        }

        std::vector<RRTNode *> near_nodes;
        near_nodes = NearC(new_point, grid_resolution_ * 3);
        ChooseParent(near_nodes, new_node_ptr);
        Rewire(near_nodes, new_node_ptr);

        nodes_ptr_.push_back(new_node_ptr);

        kdtree_flann_.setInputCloud(GetNodesCoordPointCloud().makeShared());
    }
}

std::vector<RRTNode *> RRTStar::NearC(const Eigen::Vector3d &new_point, const double search_radius) {
    std::vector<int> search_indices;
    std::vector<float> search_distances;
    pcl::PointXYZ point_xyz(new_point.cast<float>().x(),
                            new_point.cast<float>().y(),
                            new_point.cast<float>().z());
    kdtree_flann_.radiusSearch(point_xyz, search_radius, search_indices, search_distances);

    std::vector<RRTNode *> near_nodes;
    near_nodes.reserve(search_indices.size());
    for (unsigned int i = 0; i < search_indices.size(); ++i) {
        near_nodes.emplace_back(nodes_ptr_[search_indices[i]]);
    }

    return near_nodes;
}

void RRTStar::ChooseParent(const std::vector<RRTNode *> &near_nodes, RRTNode *const &new_node) {
    double min_length = std::numeric_limits<double>::max();
    RRTNode *parent_node;
    for (unsigned int i = 0; i < near_nodes.size(); ++i) {
        new_node->parent_ = near_nodes[i];

        double length = GetPathLength(start_node_, new_node);
        if (length < min_length) {
            min_length = length;
            parent_node = near_nodes[i];
        }
    }

    new_node->parent_ = parent_node;
}

void RRTStar::Rewire(std::vector<RRTNode *> &near_nodes, const RRTNode *const &new_node) {
    for (unsigned int i = 0; i < near_nodes.size(); ++i) {
        double dist_before = GetPathLength(start_node_, near_nodes[i]);
        double dist_after = GetPathLength(start_node_, new_node) +
                            (new_node->coordinate_ - near_nodes[i]->coordinate_).norm();

        if (dist_after < dist_before) {
            near_nodes[i]->parent_ = const_cast<RRTNode *>(new_node);
        }
    }
}

double RRTStar::GetPathLength(const RRTNode *start_node_ptr, const RRTNode *end_node_ptr) {
    double length = 0.0;

    const RRTNode *temp_node_ptr = end_node_ptr;
    while (temp_node_ptr->parent_ != nullptr && temp_node_ptr != start_node_ptr) {
        length += (temp_node_ptr->coordinate_ - temp_node_ptr->parent_->coordinate_).norm();
        temp_node_ptr = temp_node_ptr->parent_;
    }

    return length;
}