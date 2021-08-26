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

}